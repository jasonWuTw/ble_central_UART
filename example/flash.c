/*## 使用者
sd_flash_page_erase範例
//## Bing
以下是一個使用sd_flash_page_erase的範例，這個函數用於擦除指定的Flash頁面：
*/

void MyFlashWrite (ble_ant_csc_t* cscdata)  
{ 
    uint32_t addr; 
    uint32_t patwr [3]; 
    uint32_t pg_size; 
    uint32_t pg_num; 

    patwr [0]=111;
    patwr [1]=222; 
    patwr [2]=333; 

    pg_size = NRF_FICR->CODEPAGESIZE; 
    pg_num = NRF_FICR->CODESIZE - 1; // 使用最後一頁Flash

    addr = (pg_size * pg_num); //


/*## 使用者

sd_flash_page_erase範例


## Bing

以下是一個使用`sd_flash_page_erase`的範例，這個函數用於擦除指定的Flash頁面：

```c*/
void MyFlashWrite (ble_ant_csc_t* cscdata)  
{ 
    uint32_t addr; 
    uint32_t patwr [3]; 
    uint32_t pg_size; 
    uint32_t pg_num; 

    patwr [0]=111;
    patwr [1]=222; 
    patwr [2]=333; 

    pg_size = NRF_FICR->CODEPAGESIZE; 
    pg_num = NRF_FICR->CODESIZE - 1; // 使用最後一頁Flash

    addr = (pg_size * pg_num); //

    // 擦除Flash頁面
    sd_flash_page_erase(pg_num);

    // 寫入資料
    sd_flash_write((uint32_t*)addr, patwr, 3);
    
    uint32_t* p_data = (uint32_t*)addr; // 創建一個指向該地址的指針
    // 現在，p_data指向的就是您寫入的數據
    uint32_t data = *p_data; // 這樣就讀取了數據
    NRF_LOG_INFO("Data at address %u: %u", addr, data);
}
/*
這個範例首先定義了一個要寫入Flash的數據patwr，然後計算出最後一頁Flash的地址addr。接著，使用sd_flash_page_erase函數擦除該頁面，最後使用sd_flash_write函數將數據寫入該頁面。請注意，這只是一個範例，實際使用時可能需要根據具體情況進行調整。例如，你可能需要檢查sd_flash_page_erase和sd_flash_write函數的返回值來確保操作成功。此外，這些操作可能需要在一個特定的系統級任務中進行，因為Flash操作可能會阻塞CPU。在使用這些函數時，請參考Nordic的SDK文檔以獲得完整的指南和注意事項。
## 使用者
修改
	NRF_LOG_INFO("NRF_FICR->CODEPAGESIZE : %s",NRF_FICR->CODEPAGESIZE)

	NRF_LOG_INFO("NRF_FICR->CODESIZE : %s",NRF_FICR->CODESIZE)
## Bing
您的代码需要一些修改，因为NRF_FICR->CODEPAGESIZE和NRF_FICR->CODESIZE是整数，而不是字符串。您应该使用%d或%u来打印它们，而不是%s。以下是修改后的代码：

NRF_LOG_INFO("NRF_FICR->CODEPAGESIZE : %d", NRF_FICR->CODEPAGESIZE);
NRF_LOG_INFO("NRF_FICR->CODESIZE : %d", NRF_FICR->CODESIZE);

这样，日志将正确地打印这些值。希望这个信息对您有所帮助！
*/