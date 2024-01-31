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
void write_file_pid(t_file_pid_gain *write_gain);
t_file_pid_gain read_file_pid();
void write_file_wall_th(t_file_wall_th *write_th);
t_file_wall_th read_file_wall_th();
void unmount_fat();