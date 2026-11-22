// FILE: libc_shim.c
//
// Size-optimized replacements for newlib string routines.
//
// Newlib's strstr uses the two-way algorithm (~1.5 KB) and its strcmp is a
// large unrolled implementation (~450 B). On the STM32F103C6 (32 KB flash)
// those bytes matter, and the short strings handled by the VSCP link
// protocol gain nothing from the fancy algorithms. These definitions
// override the libc archive versions at link time.

#include <string.h>

char *
strstr(const char *haystack, const char *needle)
{
  if (!*needle) {
    return (char *) haystack;
  }
  for (; *haystack; haystack++) {
    const char *h = haystack;
    const char *n = needle;
    while (*h && *n && (*h == *n)) {
      h++;
      n++;
    }
    if (!*n) {
      return (char *) haystack;
    }
  }
  return NULL;
}

int
strcmp(const char *s1, const char *s2)
{
  while (*s1 && (*s1 == *s2)) {
    s1++;
    s2++;
  }
  return (unsigned char) *s1 - (unsigned char) *s2;
}
