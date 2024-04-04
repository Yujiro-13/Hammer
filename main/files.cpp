#include "files.hpp"

#define MAZESIZE_X 32
#define MAZESIZE_Y 32

const char *PID_FILE_TAG = "file_pid";
const char *WALL_TH_FILE_TAG = "file_wall_th";
const char *MAP_FILE_TAG = "file_map";
const char *CENTER_VALUE_TAG = "center";
const char *PARTITION_LABEL = "storage";
const std::string BASE_PATH = "/param";

/* ＊注意!! ファイル名が8文字を超えるものは、エラーで生成されない（設定で変更可） */
const std::string PID_FILE_PATH = BASE_PATH + "/pid.txt";
const std::string WALL_TH_FILE_PATH = BASE_PATH + "/wall_th.txt";
const std::string MAP_FILE_PATH = BASE_PATH + "/map.txt";
const std::string CENTER_VALUE_PATH = BASE_PATH + "/center.txt";
const esp_vfs_fat_mount_config_t MOUNT_CONFIG = {

    .format_if_mount_failed = true,
    .max_files = 4,
    .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
};

wl_handle_t wl_handle = WL_INVALID_HANDLE;

void init_files()
{
    esp_err_t err = esp_vfs_fat_spiflash_mount(
        BASE_PATH.c_str(),
        PARTITION_LABEL,
        &MOUNT_CONFIG,
        &wl_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(PID_FILE_TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(PID_FILE_TAG, "init file");
}

void write_file_pid(t_file_pid_gain *write_gain)
{
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->speed_Kp);
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->speed_Ki);
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->speed_Kd);
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->ang_vel_Kp);
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->ang_vel_Ki);
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->ang_vel_Kd);
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->wall_Kp);
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->wall_Ki);
    ESP_LOGI(PID_FILE_TAG, "write file : %lf", write_gain->wall_Kd);

    std::ofstream ffile(PID_FILE_PATH, std::ios::out);
    if (ffile.fail())
    {
        ESP_LOGE(PID_FILE_TAG, "Failed to open %s for writing", PID_FILE_PATH.c_str());
        // return;
    }
    else
    {
        ffile << write_gain->speed_Kp << std::endl;
        ffile << write_gain->speed_Ki << std::endl;
        ffile << write_gain->speed_Kd << std::endl;
        ffile << write_gain->ang_vel_Kp << std::endl;
        ffile << write_gain->ang_vel_Ki << std::endl;
        ffile << write_gain->ang_vel_Kd << std::endl;
        ffile << write_gain->wall_Kp << std::endl;
        ffile << write_gain->wall_Ki << std::endl;
        ffile << write_gain->wall_Kd << std::endl;

        ffile.close();
    }
}

t_file_pid_gain read_file_pid()
{
    t_file_pid_gain pid_gain;

    ESP_LOGI(PID_FILE_TAG, "Opening file");
    std::ifstream ifile(PID_FILE_PATH, std::ios::in);
    if (ifile.fail())
    {
        std::cerr << "Failed to open " << PID_FILE_PATH << " for reading" << std::endl;
    }
    else
    {
        std::string line;
        int line_number = 0; // 行数をカウントする変数

        while (getline(ifile, line))
        {
            // 行ごとに処理

            // ラインから値を抽出
            std::istringstream iss(line);

            // 例：speed_Kpを抽出
            if (line_number == 0)
            {
                if (!(iss >> pid_gain.speed_Kp))
                {
                    std::cerr << "Error parsing speed_Kp from line " << line_number + 1 << std::endl;
                }
            }

            // 他のメンバーを抽出
            if (line_number == 1)
            {
                if (!(iss >> pid_gain.speed_Ki))
                {
                    std::cerr << "Error parsing another_member from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 2)
            {
                if (!(iss >> pid_gain.speed_Kd))
                {
                    std::cerr << "Error parsing another_member from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 3)
            {
                if (!(iss >> pid_gain.ang_vel_Kp))
                {
                    std::cerr << "Error parsing another_member from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 4)
            {
                if (!(iss >> pid_gain.ang_vel_Ki))
                {
                    std::cerr << "Error parsing another_member from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 5)
            {
                if (!(iss >> pid_gain.ang_vel_Kd))
                {
                    std::cerr << "Error parsing another_member from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 6)
            {
                if (!(iss >> pid_gain.wall_Kp))
                {
                    std::cerr << "Error parsing another_member from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 7)
            {
                if (!(iss >> pid_gain.wall_Ki))
                {
                    std::cerr << "Error parsing another_member from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 8)
            {
                if (!(iss >> pid_gain.wall_Kd))
                {
                    std::cerr << "Error parsing another_member from line " << line_number + 1 << std::endl;
                }
            }

            line_number++;
        }

        ESP_LOGI(PID_FILE_TAG, "read file speed_Kp : %lf", pid_gain.speed_Kp);
        ESP_LOGI(PID_FILE_TAG, "read file speed_Ki : %lf", pid_gain.speed_Ki);
        ESP_LOGI(PID_FILE_TAG, "read file speed_Kd : %lf", pid_gain.speed_Kd);
        ESP_LOGI(PID_FILE_TAG, "read file ang_vel_Kp: %lf", pid_gain.ang_vel_Kp);
        ESP_LOGI(PID_FILE_TAG, "read file ang_vel_Ki: %lf", pid_gain.ang_vel_Ki);
        ESP_LOGI(PID_FILE_TAG, "read file ang_vel_Kd: %lf", pid_gain.ang_vel_Kd);
        ESP_LOGI(PID_FILE_TAG, "read file wall_Kp: %lf", pid_gain.wall_Kp);
        ESP_LOGI(PID_FILE_TAG, "read file wall_Ki: %lf", pid_gain.wall_Ki);
        ESP_LOGI(PID_FILE_TAG, "read file wall_Kd: %lf", pid_gain.wall_Kd);
    }
    ifile.close();

    return pid_gain;
}

void write_file_wall_th(t_file_wall_th *write_wall_th)
{
    ESP_LOGI(WALL_TH_FILE_TAG, "write file : %d", write_wall_th->th_wall_fl);
    ESP_LOGI(WALL_TH_FILE_TAG, "write file : %d", write_wall_th->th_wall_l);
    ESP_LOGI(WALL_TH_FILE_TAG, "write file : %d", write_wall_th->th_wall_r);
    ESP_LOGI(WALL_TH_FILE_TAG, "write file : %d", write_wall_th->th_wall_fr);
    ESP_LOGI(WALL_TH_FILE_TAG, "write file : %d", write_wall_th->th_control_l);
    ESP_LOGI(WALL_TH_FILE_TAG, "write file : %d", write_wall_th->th_control_r);
    ESP_LOGI(WALL_TH_FILE_TAG, "write file : %d", write_wall_th->ref_l);
    ESP_LOGI(WALL_TH_FILE_TAG, "write file : %d", write_wall_th->ref_r);

    std::ofstream ffile(WALL_TH_FILE_PATH, std::ios::out);
    if (ffile.fail())
    {
        ESP_LOGE(WALL_TH_FILE_TAG, "Failed to open %s for writing", WALL_TH_FILE_PATH.c_str());
        // return;
    }
    else
    {
        ffile << write_wall_th->th_wall_fl << std::endl;
        ffile << write_wall_th->th_wall_l << std::endl;
        ffile << write_wall_th->th_wall_r << std::endl;
        ffile << write_wall_th->th_wall_fr << std::endl;
        ffile << write_wall_th->th_control_l << std::endl;
        ffile << write_wall_th->th_control_r << std::endl;
        ffile << write_wall_th->ref_l << std::endl;
        ffile << write_wall_th->ref_r << std::endl;

        ffile.close();
    }
}

t_file_wall_th read_file_wall_th()
{
    t_file_wall_th th_value;

    ESP_LOGI(WALL_TH_FILE_TAG, "Opening file");
    std::ifstream ifile(WALL_TH_FILE_PATH, std::ios::in);
    if (ifile.fail())
    {
        std::cerr << "Failed to open " << WALL_TH_FILE_PATH << " for reading" << std::endl;
    }
    else
    {
        std::string line;
        int line_number = 0; // 行数をカウントする変数

        while (getline(ifile, line))
        {
            // 行ごとに処理

            // ラインから値を抽出
            std::istringstream iss(line);

            // 例：speed_Kpを抽出
            if (line_number == 0)
            {
                if (!(iss >> th_value.th_wall_fl))
                {
                    std::cerr << "Error parsing th_wall_fl from line " << line_number + 1 << std::endl;
                }
            }

            // 他のメンバーを抽出
            if (line_number == 1)
            {
                if (!(iss >> th_value.th_wall_l))
                {
                    std::cerr << "Error parsing th_wall_l from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 2)
            {
                if (!(iss >> th_value.th_wall_r))
                {
                    std::cerr << "Error parsing th_wall_r from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 3)
            {
                if (!(iss >> th_value.th_wall_fr))
                {
                    std::cerr << "Error parsing th_wall_fr from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 4)
            {
                if (!(iss >> th_value.th_control_l))
                {
                    std::cerr << "Error parsing th_control_l from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 5)
            {
                if (!(iss >> th_value.th_control_r))
                {
                    std::cerr << "Error parsing th_control_r from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 6)
            {
                if (!(iss >> th_value.ref_l))
                {
                    std::cerr << "Error parsing ref_l from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 7)
            {
                if (!(iss >> th_value.ref_r))
                {
                    std::cerr << "Error parsing ref_r from line " << line_number + 1 << std::endl;
                }
            }

            line_number++;
        }

        ESP_LOGI(WALL_TH_FILE_TAG, "read file th_wall_fl : %d", th_value.th_wall_fl);
        ESP_LOGI(WALL_TH_FILE_TAG, "read file th_wall_l : %d", th_value.th_wall_l);
        ESP_LOGI(WALL_TH_FILE_TAG, "read file th_wall_r : %d", th_value.th_wall_r);
        ESP_LOGI(WALL_TH_FILE_TAG, "read file th _wall_fr: %d", th_value.th_wall_fr);
        ESP_LOGI(WALL_TH_FILE_TAG, "read file th_control_l: %d", th_value.th_control_l);
        ESP_LOGI(WALL_TH_FILE_TAG, "read file th_control_r: %d", th_value.th_control_r);
        ESP_LOGI(WALL_TH_FILE_TAG, "read file ref_l: %d", th_value.ref_l);
        ESP_LOGI(WALL_TH_FILE_TAG, "read file ref_r: %d", th_value.ref_r);
    }
    ifile.close();

    return th_value;
}



void map_write(t_map *map)
{
    std::ofstream ffile(MAP_FILE_PATH, std::ios::out);
    if (ffile.fail())
    {
        ESP_LOGE(MAP_FILE_TAG, "Failed to open %s for writing", "/map.txt");
        // return;
    }
    else
    {
        ffile.write(reinterpret_cast<const char *>(map), sizeof(t_map));

        ffile.close();
    }
}

t_map map_read()
{
    t_map map;

    ESP_LOGI(MAP_FILE_TAG, "Opening file");
    std::ifstream ifile(MAP_FILE_PATH, std::ios::in);
    if (ifile.fail())
    {
        std::cerr << "Failed to open " << MAP_FILE_PATH << " for reading" << std::endl;
    }
    else
    {
        ESP_LOGI(MAP_FILE_TAG, "read file");
        ifile.read(reinterpret_cast<char *>(&map), sizeof(t_map));  // ここでスタックオーバーフロー 追記:原因忘れた 確か最短走行の前に呼び出そうとすると起きてた気がする
                                                                    // 追記2:多分 呼び出す箇所でInitMazeが正しく呼べていなかったことが原因。詳しくは、fast.cppのコメント参照
        ifile.close();

        ESP_LOGI(MAP_FILE_TAG, "read file wall north : %d", map.wall[0][0].north);
        ESP_LOGI(MAP_FILE_TAG, "read file wall east: %d", map.wall[0][0].east);
        ESP_LOGI(MAP_FILE_TAG, "read file wall south : %d", map.wall[0][0].south);
        ESP_LOGI(MAP_FILE_TAG, "read file wall west: %d", map.wall[0][0].west);
    }

    return map;
}

void write_file_center_sens_val(t_file_center_sens_value *center_value)
{
    ESP_LOGI(CENTER_VALUE_TAG, "write file : %d", center_value->front_l);
    ESP_LOGI(CENTER_VALUE_TAG, "write file : %d", center_value->front_r);
    ESP_LOGI(CENTER_VALUE_TAG, "write file : %d", center_value->left_fl);
    ESP_LOGI(CENTER_VALUE_TAG, "write file : %d", center_value->left_fr);
    ESP_LOGI(CENTER_VALUE_TAG, "write file : %d", center_value->right_fl);
    ESP_LOGI(CENTER_VALUE_TAG, "write file : %d", center_value->right_fr);
    ESP_LOGI(CENTER_VALUE_TAG, "write file : %d", center_value->rear_fl);
    ESP_LOGI(CENTER_VALUE_TAG, "write file : %d", center_value->rear_fr);

    std::ofstream ffile(CENTER_VALUE_PATH, std::ios::out);
    if (ffile.fail())
    {
        ESP_LOGE(CENTER_VALUE_TAG, "Failed to open %s for writing. Error: %s", CENTER_VALUE_PATH.c_str(), strerror(errno));
        // return;
    }
    else
    {
        ffile << center_value->front_l << std::endl;
        ffile << center_value->front_r << std::endl;
        ffile << center_value->left_fl << std::endl;
        ffile << center_value->left_fr << std::endl;
        ffile << center_value->right_fl << std::endl;
        ffile << center_value->right_fr << std::endl;
        ffile << center_value->rear_fl << std::endl;
        ffile << center_value->rear_fr << std::endl;

        ffile.close();
    }
}

t_file_center_sens_value read_file_center_sens_val()
{
    t_file_center_sens_value ce_value;

    ESP_LOGI(CENTER_VALUE_TAG, "Opening file");
    std::ifstream ifile(CENTER_VALUE_PATH, std::ios::in);
    if (ifile.fail())
    {
        ESP_LOGE(CENTER_VALUE_TAG, "Failed to open %s for reading. Error: %s", CENTER_VALUE_PATH.c_str(), strerror(errno));
    }
    else
    {
        std::string line;
        int line_number = 0; // 行数をカウントする変数

        while (getline(ifile, line))
        {
            // 行ごとに処理

            // ラインから値を抽出
            std::istringstream iss(line);

            // 例：speed_Kpを抽出
            if (line_number == 0)
            {
                if (!(iss >> ce_value.front_l))
                {
                    std::cerr << "Error parsing front_l from line " << line_number + 1 << std::endl;
                }
            }

            // 他のメンバーを抽出
            if (line_number == 1)
            {
                if (!(iss >> ce_value.front_r))
                {
                    std::cerr << "Error parsing front_r from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 2)
            {
                if (!(iss >> ce_value.left_fl))
                {
                    std::cerr << "Error parsing left_fl from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 3)
            {
                if (!(iss >> ce_value.left_fr))
                {
                    std::cerr << "Error parsing left_fr from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 4)
            {
                if (!(iss >> ce_value.right_fl))
                {
                    std::cerr << "Error parsing right_fl from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 5)
            {
                if (!(iss >> ce_value.right_fr))
                {
                    std::cerr << "Error parsing right_fr from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 6)
            {
                if (!(iss >> ce_value.rear_fl))
                {
                    std::cerr << "Error parsing rear_fl from line " << line_number + 1 << std::endl;
                }
            }

            if (line_number == 7)
            {
                if (!(iss >> ce_value.rear_fr))
                {
                    std::cerr << "Error parsing rear_fr from line " << line_number + 1 << std::endl;
                }
            }

            line_number++;
        }

        ESP_LOGI(CENTER_VALUE_TAG, "read file front_l : %d", ce_value.front_l);
        ESP_LOGI(CENTER_VALUE_TAG, "read file front_r : %d", ce_value.front_r);
        ESP_LOGI(CENTER_VALUE_TAG, "read file left_fl : %d", ce_value.left_fl);
        ESP_LOGI(CENTER_VALUE_TAG, "read file left_fr : %d", ce_value.left_fr);
        ESP_LOGI(CENTER_VALUE_TAG, "read file right_fl: %d", ce_value.right_fl);
        ESP_LOGI(CENTER_VALUE_TAG, "read file right_fr: %d", ce_value.right_fr);
        ESP_LOGI(CENTER_VALUE_TAG, "read file rear_fl: %d", ce_value.rear_fl);
        ESP_LOGI(CENTER_VALUE_TAG, "read file rear_fr: %d", ce_value.rear_fr);
    }

    ifile.close();

    return ce_value;
}

void unmount_fat()
{
    ESP_LOGI(BASE_PATH.c_str(), "Umounting FATFS");
    esp_vfs_fat_spiflash_unmount(BASE_PATH.c_str(), wl_handle);
}
