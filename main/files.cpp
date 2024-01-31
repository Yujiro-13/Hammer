#include "files.hpp"

const char *TAG = "file_pid";
const char *PARTITION_LABEL = "storage";
const std::string BASE_PATH = "/param";
const std::string FILE_PATH = BASE_PATH + "/pid.txt";
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
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}

void write_file_pid(t_file_pid_gain *write_gain)
{
    ESP_LOGI(TAG, "write file : %lf", write_gain->speed_Kp);
    ESP_LOGI(TAG, "write file : %lf", write_gain->speed_Ki);
    ESP_LOGI(TAG, "write file : %lf", write_gain->speed_Kd);
    ESP_LOGI(TAG, "write file : %lf", write_gain->ang_vel_Kp);
    ESP_LOGI(TAG, "write file : %lf", write_gain->ang_vel_Ki);
    ESP_LOGI(TAG, "write file : %lf", write_gain->ang_vel_Kd);
    ESP_LOGI(TAG, "write file : %lf", write_gain->wall_Kp);
    ESP_LOGI(TAG, "write file : %lf", write_gain->wall_Ki);
    ESP_LOGI(TAG, "write file : %lf", write_gain->wall_Kd);

    std::ofstream ffile(FILE_PATH, std::ios::out);
    if (ffile.fail())
    {
        ESP_LOGE(TAG, "Failed to open %s for writing", FILE_PATH.c_str());
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

    ESP_LOGI(TAG, "Opening file");
    std::ifstream ifile(FILE_PATH, std::ios::in);
    if (ifile.fail())
    {
        std::cerr << "Failed to open " << FILE_PATH << " for reading" << std::endl;
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

        ESP_LOGI(TAG, "read file speed_Kp : %lf", pid_gain.speed_Kp);
        ESP_LOGI(TAG, "read file speed_Ki : %lf", pid_gain.speed_Ki);
        ESP_LOGI(TAG, "read file speed_Kd : %lf", pid_gain.speed_Kd);
        ESP_LOGI(TAG, "read file ang_vel_Kp: %lf", pid_gain.ang_vel_Kp);
        ESP_LOGI(TAG, "read file ang_vel_Ki: %lf", pid_gain.ang_vel_Ki);
        ESP_LOGI(TAG, "read file ang_vel_Kd: %lf", pid_gain.ang_vel_Kd);
        ESP_LOGI(TAG, "read file wall_Kp: %lf", pid_gain.wall_Kp);
        ESP_LOGI(TAG, "read file wall_Ki: %lf", pid_gain.wall_Ki);
        ESP_LOGI(TAG, "read file wall_Kd: %lf", pid_gain.wall_Kd);
    }
    ifile.close();

    return pid_gain;
}

void unmount_fat()
{
    ESP_LOGI(TAG, "Umounting FATFS");
    esp_vfs_fat_spiflash_unmount(BASE_PATH.c_str(), wl_handle);
}