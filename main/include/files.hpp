#ifndef FILES_HPP
#define FILES_HPP

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

void map_write(t_map *map);
t_map map_read();
void write_file_center_sens_val(t_file_center_sens_value *write_val);
t_file_center_sens_value read_file_center_sens_val();

void unmount_fat();


#endif // FILES_HPP