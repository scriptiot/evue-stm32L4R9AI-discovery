#include "evm.h"
#include "evm_main.h"
#include "lvgl.h"
#include "lvgl_main.h"
#include "ecma.h"
#include "js_app.h"
#include "ff.h"
#include "malloc.h"

FATFS fs;
char * vm_load_file(evm_t * e, char *filename){
    FIL file;
	int size;
    char *buffer = NULL;

    FRESULT res = f_open(&file, filename, FA_READ | FA_OPEN_EXISTING);
    if( res == FR_OK ) {
		size = f_size(&file);
		UINT br;
		evm_val_t * b = evm_buffer_create(e, sizeof(uint8_t)*size + 1);
		buffer = (char*)evm_buffer_addr(b);
		res = f_read(&file, buffer, size, &br);
		buffer[size] = 0;
		if( res != FR_OK || br != size){
			return NULL;
		}
		f_close(&file);
		return buffer;
	}
	f_close(&file);
	return NULL;
}

const char * vm_load(evm_t * e, char * path, int type)
{
    int file_name_len = strlen(path) + 1;
    char* buffer = NULL;
    if(type == EVM_LOAD_MAIN){ //main文件加载路径
        sprintf(e->file_name, "%s", path);
        buffer = vm_load_file(e, path);
    } else { //模块加载路径
        sprintf(e->file_name, "%s", path);
        buffer = vm_load_file(e, path);
    }
    return buffer;
}


void * vm_malloc(int size)
{
    void * m = mymalloc(SRAMEX, size);
    if(m) mymemset(m, 0 ,size);
    return m;
}

void vm_free(void * mem)
{
    if(mem) myfree(SRAMEX, mem);
}


static char router_app_dir[PATH_LENGTH_MAX];

int evm_main(char * appdir)
{
    if (appdir == NULL){
        sprintf(router_app_dir, "%s", "0:/");
    }else{
        sprintf(router_app_dir, "%s", appdir);
    }
	
	FRESULT res_sd = f_mount(&fs,"0:",1);

    evm_register_free((intptr_t)vm_free);
    evm_register_malloc((intptr_t)vm_malloc);
    evm_register_print((intptr_t)printf);
    evm_register_file_load((intptr_t)vm_load);
	
	lvgl_main();
    evm_t * env = js_app_on_create(router_app_dir);

    while(1){
        HAL_Delay(10);
        lv_task_handler();
        ecma_timeout_poll(env);
    }
    return 0;
}

void SysTick_Handler(void)
{
  HAL_IncTick();
  lv_tick_inc(1);
}
