#include "util.h"

namespace util_ns
{

        // interfaces for computing memory usuage
        int parse_line_for_memory(char* line)
        {
                // This assumes that a digit will be found and the line ends in " Kb".
                int i = strlen(line);
                const char* p = line;
                while (*p <'0' || *p > '9')
                        p++;
                line[i-3] = '\0';
                i = atoi(p);
                return i;
        }

	int get_value_for_memory()
        { //Note: this value is in KB!
                FILE* file = fopen("/proc/self/status", "r");
                int result = -1;
                char line[128];

                while (fgets(line, 128, file) != NULL)
                {
                        if (strncmp(line, "VmRSS:", 6) == 0)
                        {
                                result = parse_line_for_memory(line);
                                break;
                        }
                }
                fclose(file);
                return result;
        }



}
