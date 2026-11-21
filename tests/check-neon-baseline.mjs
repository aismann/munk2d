// Compare ordinary baseline observations between independent NEON-included and
// NEON-omitted builds. Do not compare baseline FMA arithmetic with a non-fused
// packet oracle, and do not normalize away any printed checkpoint differences.
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';

function validate(text) {
  const data = JSON.parse(text);
  assert.equal(data.benchmarks.length, 26, 'all 26 reference scenes required');
  for (const scene of data.benchmarks) {
    const steps = [...new Set([0, 1, 10, 100, scene.steps])].sort((a, b) => a - b);
    assert.deepEqual(scene.checkpoints.map(checkpoint => checkpoint.step), steps,
      `${scene.benchmark}: all requested checkpoints required`);
    assert.equal(scene.simulated_steps, scene.steps, `${scene.benchmark}: complete run required`);
    for (const checkpoint of scene.checkpoints) {
      assert.equal(checkpoint.invalid_values, 0, `${scene.benchmark}: non-finite state`);
    }
  }
  return data;
}

const [mode, first, second] = process.argv.slice(2);
if (mode === 'capture' && first && second) {
  const result = spawnSync(resolve(first), [
    '--contact-solver', 'baseline', '--profile', 'reference',
    '--summary-json', '--checkpoints', '0,1,10,100,final',
  ], { encoding: 'utf8', timeout: 300_000, maxBuffer: 16 * 1024 * 1024 });
  if (result.error) throw result.error;
  assert.equal(result.status, 0, result.stderr);
  assert.equal(result.stderr, '', 'unexpected benchmark diagnostics');
  const data = validate(result.stdout);
  await mkdir(dirname(resolve(second)), { recursive: true });
  await writeFile(second, result.stdout);
  const count = data.benchmarks.reduce((sum, scene) => sum + scene.checkpoints.length, 0);
  console.log(`PASS: captured 26 baseline reference scenes / ${count} finite checkpoints`);
} else if (mode === 'compare' && first && second) {
  const [before, after] = await Promise.all([readFile(first, 'utf8'), readFile(second, 'utf8')]);
  validate(before); validate(after);
  assert.ok(after === before, 'NEON availability changed ordinary baseline checkpoint observations');
  console.log('PASS: NEON included/omitted baseline observations match byte-for-byte in all 26 reference scenes');
} else {
  throw new Error('usage: node tests/check-neon-baseline.mjs capture EXECUTABLE OUTPUT.json | compare BEFORE.json AFTER.json');
}
