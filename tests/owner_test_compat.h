#ifndef MUNK2D_OWNER_TEST_COMPAT_H
#define MUNK2D_OWNER_TEST_COMPAT_H

/* MSVC builds the library (including private symbols) as C++. Keep these
 * implementation-including tests usable in both that mode and ordinary C. */
#ifdef __cplusplus
#define OWNER_TEST_LITERAL(type) type
#else
#define OWNER_TEST_LITERAL(type) (type)
#endif

#endif
