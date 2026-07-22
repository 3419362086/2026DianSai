#include "flash_app.h"

uint8_t flash_write_buffer[SPI_FLASH_PAGE_SIZE];
uint8_t flash_read_buffer[SPI_FLASH_PAGE_SIZE];

unsigned char start_count = 0;

void Flash_Init(void)
{
  Uart_Printf(DEBUG_UART, "Flash_Init ......\r\n");
  Flash_SPI_Test();
    
//  spi_flash_init();
  
  spi_flash_buffer_read(&start_count, 0x1000, 1);
  start_count = start_count + 1;
  
  spi_flash_sector_erase(0x1000);
  spi_flash_buffer_write(&start_count, 0x1000, 1);
  
//  Uart_Printf(DEBUG_UART, "start_count:%d\r\n", (int)start_count);
}

void Flash_Task(void)
{

}

void Flash_SPI_Test(void) 
{
    uint32_t flash_id;
//    uint8_t write_buffer[SPI_FLASH_PAGE_SIZE];
//    uint8_t read_buffer[SPI_FLASH_PAGE_SIZE];
    uint32_t test_addr = 0x000000; // 测试地址，选择一个扇区的起始

    Uart_Printf(DEBUG_UART, "SPI FLASH Test Start\r\n");

    // 1. 初始化SPI Flash驱动 (主要是CS引脚状态)
    spi_flash_init();
    Uart_Printf(DEBUG_UART, "SPI Flash Initialized.\r\n");

    // 2. 读取Flash ID
    flash_id = spi_flash_read_id();
    Uart_Printf(DEBUG_UART, "Flash ID: 0x%lX\r\n", flash_id);
    // 你可以根据你的芯片手册检查ID是否正确，例如 GD25Q64的ID可能是 0xC84017

//    // 3. 擦除一个扇区 (大小通常为4KB)
//    // 注意：擦除操作耗时较长
//    Uart_Printf(DEBUG_UART, "Erasing sector at address 0x%lX...\r\n", test_addr);
    spi_flash_sector_erase(test_addr);
//    Uart_Printf(DEBUG_UART, "Sector erased.\r\n");

//    // (可选) 校验擦除：读取一页数据，检查是否全为0xFF
//    spi_flash_buffer_read(read_buffer, test_addr, SPI_FLASH_PAGE_SIZE);
//    int erased_check_ok = 1;
//    for (int i = 0; i < SPI_FLASH_PAGE_SIZE; i++) {
//        if (read_buffer[i] != 0xFF) {
//            erased_check_ok = 0;
//            break;
//        }
//    }
//    if (erased_check_ok) {
//        Uart_Printf(DEBUG_UART, "Erase check PASSED. Sector is all 0xFF.\r\n");
//    } else {
//        Uart_Printf(DEBUG_UART, "Erase check FAILED.\r\n");
//    }

//    // 4. 准备写入数据 (写入一页)
//    const char* message = "Hello from STM32 to SPI FLASH! Microunion Studio Test - 12345.";
//    uint16_t data_len = strlen(message);
//    if (data_len >= SPI_FLASH_PAGE_SIZE) {
//        data_len = SPI_FLASH_PAGE_SIZE -1; // 确保不超过页大小
//    }
//    memset(write_buffer, 0, SPI_FLASH_PAGE_SIZE);
//    memcpy(write_buffer, message, data_len);
//    write_buffer[data_len] = '\0'; // 确保字符串结束

//    Uart_Printf(DEBUG_UART, "Writing data to address 0x%lX: \"%s\"\r\n", test_addr, write_buffer);
//    // 使用 spi_flash_buffer_write (可以处理跨页，但这里只写一页内)
//    // 或者直接用 spi_flash_page_write 如果确定在一页内
//    spi_flash_buffer_write(write_buffer, test_addr, SPI_FLASH_PAGE_SIZE); // 写入整页数据，包括填充的0
//    // spi_flash_page_write(write_buffer, test_addr, data_len + 1); // 只写入有效数据
//    Uart_Printf(DEBUG_UART, "Data written.\r\n");

//    // 5. 读取写入的数据
//    Uart_Printf(DEBUG_UART, "Reading data from address 0x%lX...\r\n", test_addr);
//    memset(read_buffer, 0, SPI_FLASH_PAGE_SIZE);
//    spi_flash_buffer_read(read_buffer, test_addr, SPI_FLASH_PAGE_SIZE);
//    Uart_Printf(DEBUG_UART, "Data read: \"%s\"\r\n", read_buffer);

//    // 6. 校验数据
//    if (memcmp(write_buffer, read_buffer, SPI_FLASH_PAGE_SIZE) == 0) {
//        Uart_Printf(DEBUG_UART, "Data VERIFIED! Write and Read successful.\r\n");
//    } else {
//        Uart_Printf(DEBUG_UART, "Data VERIFICATION FAILED!\r\n");
//    }

    Uart_Printf(DEBUG_UART, "SPI FLASH Test End\r\n");
}
