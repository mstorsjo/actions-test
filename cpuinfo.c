#include <stdio.h>

int main(int argc, char* argv[]) {
    FILE *f = fopen("/proc/cpuinfo", "r");
    char line[1000];
    while (fgets(line, sizeof(line), f)) {
        printf("%s", line);
    }
    fclose(f);
    return 0;
}
