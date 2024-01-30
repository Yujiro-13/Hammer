#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "structs.hpp"

void init_files();
void write_files(t_file_pid_gain *write_gain);
t_file_pid_gain read_file_pid();
void unmount_fat();