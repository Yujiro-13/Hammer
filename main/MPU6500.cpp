#include "include/Driver/MPU6500.hpp"

MPU6500::MPU6500(spi_host_device_t bus, gpio_num_t cs){
    
    // バスの初期化
    memset(&bus_imu, 0, sizeof(bus_imu));
    bus_imu.mosi_io_num = IMU_MOSI;
    bus_imu.miso_io_num = IMU_MISO;
    bus_imu.sclk_io_num = IMU_CLK;
    bus_imu.quadwp_io_num = -1;
    bus_imu.quadhd_io_num = -1;
    bus_imu.max_transfer_sz = 16;
    bus_imu.intr_flags = 0;

    //ret = spi_bus_initialize(SPI2_HOST, &bus_imu, SPI_DMA_CH_AUTO);
    //ESP_ERROR_CHECK(ret);

    // デバイスの初期化
    memset(&dev_imu,0,sizeof(dev_imu));
    dev_imu.clock_speed_hz = 7*1000*1000;
    dev_imu.mode = 3;
    dev_imu.spics_io_num = cs;
    dev_imu.queue_size = 7;

    //err = spi_bus_add_device(bus,&dev_imu,&_spi);
    //ESP_ERROR_CHECK(err);

    write(0x6B,0b10000000);
    write(0x6B,0b00001001); //温度センサ無効

    uint8_t who = whoami();
    if(who != MPU6500_WHO_AM_I){ //MPU6500 : read(0x75)  ICM20648 : read(0x00)
        printf("%d\r\n",who);
        printf("failed\r\n");
        // while (1) {} // Add a placeholder statement
        return;
    }
    
    if(changesens(2,1) == -1){
        while (1) {} // Add a placeholder statement
    }
}

MPU6500::~MPU6500(){}

uint8_t MPU6500::read(uint8_t reg){
    esp_err_t err;
    spi_transaction_t cmd;
    uint16_t tx = (reg | MPU6500_READ_FLAG) << 8;
    //tx = SPI_SWAP_DATA_TX(tx,16);

    memset(&cmd,0,sizeof(cmd));
    //cmd.flags = SPI_TRANS_USE_RXDATA;
    cmd.length = 16;
    cmd.tx_buffer = &tx;
    //err = spi_device_polling_transmit(_spi,&cmd);
    //assert(err == ESP_OK);

    //uint8_t rx = 
        //SPI_SWAP_DATA_RX(*(uint16_t*)cmd.rx_data,16) & 0xFF;

    //return rx;
    return 0;
}

uint16_t MPU6500::read16(uint8_t reg){
    esp_err_t err;
    spi_transaction_t cmd;

    memset(&cmd,0,sizeof(cmd));
    //cmd.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    cmd.length = 24;
    cmd.tx_data[0] = (reg | MPU6500_READ_FLAG);
    cmd.tx_data[1] = 0;
    cmd.tx_data[2] = 0;
    //err = spi_device_polling_transmit(_spi,&cmd);
    //assert(err == ESP_OK);

    return cmd.rx_data[1] << 8 | cmd.rx_data[2];

}


void MPU6500::write(uint8_t reg,uint8_t data){
    esp_err_t err;
    spi_transaction_t cmd;

    memset(&cmd,0,sizeof(cmd));
    //cmd.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    cmd.length = 16;
    cmd.tx_data[0] = reg;
    cmd.tx_data[1] = data;
    //err = spi_device_polling_transmit(_spi,&cmd);
    //assert(err == ESP_OK);
    //Delay挟まないとうまくいかないことがある
    //vTaskDelay(10/portTICK_PERIOD_MS);
    return;
}



int MPU6500::changesens(uint8_t _gyro,uint8_t _accel){
    if(_gyro > 0b11 || _accel > 0b11) return -1;

    write(GYRO_CONFIG,_gyro << GYRO_FS_SEL);
    write(ACCEL_CONFIG,_accel << ACCEL_FS_SEL);

    bool _verify = ((read(GYRO_CONFIG) >> GYRO_FS_SEL) & 0b11) == _gyro 
    && ((read(ACCEL_CONFIG) >> ACCEL_FS_SEL) & 0b11) == _accel;


    if(!_verify) return -1;
    
    switch (_gyro){
    case 0:
        gyro_sensitivity = 250.0 /32768.;
        break;
    case 1:
        gyro_sensitivity = 500.0 /32768.;
        break;
    case 2:
        gyro_sensitivity = 1000.0/32768.;
        break;
    case 3:
        gyro_sensitivity = 2000.0 /32768.;
        break;
    default:
        break;
    }
    switch (_accel){
    case 0:
        accel_sensitivity = 2.0 / 32768.;
        break;
    case 1:
        accel_sensitivity = 4.0 / 32768.;
        break;
    case 2:
        accel_sensitivity = 8.0 / 32768.;
        break;
    case 3:
        accel_sensitivity = 16.0 / 32768.;
        break;
    
    default:
        break;
    }

    return 0;
}

float MPU6500::surveybias(int reftime){
    in_survaeybias = true; //  バイアスサーベイ中フラグを立てる
    //vTaskDelay(1000/portTICK_PERIOD_MS); //  100ms待つ
    float r_yaw_ref_tmp = 0;
    for(uint16_t i = 0; i < reftime; i++){
        r_yaw_ref_tmp += gyroZ();
        //vTaskDelay(1/portTICK_PERIOD_MS);
    }
    in_survaeybias = false; 
    return (float)(r_yaw_ref_tmp / reftime);
}

uint8_t MPU6500::whoami(){
    return read(0x75);  //  0x00
}
int16_t MPU6500::accelX_raw(){
    return read16(0x3B);    //  0x3D
}
int16_t MPU6500::accelY_raw(){
    return read16(0x3D);    //  0x3F
}
int16_t MPU6500::accelZ_raw(){
    return read16(0x3F);    //  0x41
}
int16_t MPU6500::gyroX_raw(){
    return read16(0x43);    //  0x43
}
int16_t MPU6500::gyroY_raw(){
    return read16(0x45);    //  0x45
}
int16_t MPU6500::gyroZ_raw(){
    return read16(0x47);    //  0x47
}
float MPU6500::accelX(){
    return (float)accelX_raw() * accel_sensitivity;
}
float MPU6500::accelY(){
    return (float)accelY_raw() * accel_sensitivity;
}
float MPU6500::accelZ(){
    return (float)accelZ_raw() * accel_sensitivity;
}
float MPU6500::gyroX(){
    return (float)gyroX_raw() * gyro_sensitivity;
}
float MPU6500::gyroY(){
    return (float)gyroY_raw() * gyro_sensitivity;
}
float MPU6500::gyroZ(){
    return (float)gyroZ_raw() * gyro_sensitivity;
}

void MPU6500::GetData(t_sens_data *_sens){
    sens = _sens;
}
