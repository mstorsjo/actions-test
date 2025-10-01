#include <stdio.h>
#include <windows.h>

int main(int argc, char * argv[])
{
  fprintf(stderr, "ExitThread\n"); fflush(stderr);
  ExitThread(0);
  fprintf(stderr, "not exited?\n"); fflush(stderr);
  return 1;
}
