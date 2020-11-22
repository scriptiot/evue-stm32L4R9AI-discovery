#include "lvgl_main.h"
#include "main.h"
#include "stm32l4r9i_discovery_ts.h"
#include "mfxstm32l152.h"
#include "lv_png.h"

static TS_StateTypeDef  TS_State;

extern void CopyInVirtualBuffer(uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize);
extern DSI_HandleTypeDef    DsiHandle;
extern void ecma_timer_poll(evm_t * e);

static lv_disp_buf_t disp_buf;
static lv_color_t * buf_1;
static lv_color_t * buf_2;

lv_disp_t *g_disp;

void my_disp_flush(lv_disp_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
	int h = area->y2 - area->y1 + 1;
	int w = area->x2 - area->x1 + 1;
	CopyInVirtualBuffer((uint32_t *)color_p, (uint32_t *)LAYER_ADDRESS, area->x1, area->y1, w, h);
	HAL_DSI_Refresh(&DsiHandle);
	g_disp = disp;
            /* Indicate you are ready with the flushing*/
}

void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{
	lv_disp_flush_ready(g_disp); 
}


void evm_lvgl_tick_inc(int x){
    lv_tick_inc(x);
}

__IO FlagStatus MfxItOccurred = RESET;
__IO uint32_t TouchEvent = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == MFX_INT_PIN)
  {

    MfxItOccurred = SET;
	  
  }
  /* Check if interrupt comes Joystick SEL */
  else if(GPIO_Pin == SEL_JOY_PIN)
  {

  }
  else
  {
    /* Unexpected case */
  }
}


void EXTI1_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(MFX_INT_PIN);
}

#define MFX_IRQ_PENDING_GPIO    0x01
#define MFX_IRQ_PENDING_IDD     0x02
#define MFX_IRQ_PENDING_ERROR   0x04
#define MFX_IRQ_PENDING_TS_DET  0x08
#define MFX_IRQ_PENDING_TS_NE   0x10
#define MFX_IRQ_PENDING_TS_TH   0x20
#define MFX_IRQ_PENDING_TS_FULL 0x40
#define MFX_IRQ_PENDING_TS_OVF  0x80

__IO uint32_t errorSrc = 0;
__IO uint32_t errorMsg = 0;
void Mfx_Event(void)
{
  uint32_t irqPending;

  /* Reset joystick state */

  irqPending = MFX_IO_Read(IO_I2C_ADDRESS, MFXSTM32L152_REG_ADR_IRQ_PENDING);

  /* GPIO IT from MFX */
  if(irqPending & MFX_IRQ_PENDING_GPIO)
  {
    uint32_t JoystickStatus;
    uint32_t statusGpio = BSP_IO_ITGetStatus(RIGHT_JOY_PIN | LEFT_JOY_PIN | UP_JOY_PIN | DOWN_JOY_PIN | TS_INT_PIN | SD_DETECT_PIN);

    TouchEvent =  statusGpio & TS_INT_PIN;

    JoystickStatus = statusGpio & (RIGHT_JOY_PIN | LEFT_JOY_PIN | UP_JOY_PIN | DOWN_JOY_PIN);

    /* Insert a little delay to avoid debounce */
    HAL_Delay(1);

    /* Clear IO Expander IT */
    BSP_IO_ITClear(statusGpio);
  }
  if(irqPending & MFX_IRQ_PENDING_ERROR)
  {
    errorSrc = MFX_IO_Read(IO_I2C_ADDRESS, MFXSTM32L152_REG_ADR_ERROR_SRC);
    errorMsg = MFX_IO_Read(IO_I2C_ADDRESS, MFXSTM32L152_REG_ADR_ERROR_MSG);
  }

  /* Ack all IRQ pending except GPIO if any */
  irqPending &= ~MFX_IRQ_PENDING_GPIO;
  if(irqPending)
  {
    MFX_IO_Write(IO_I2C_ADDRESS, MFXSTM32L152_REG_ADR_IRQ_ACK, irqPending);
  }

  MfxItOccurred = RESET;

  /* Re-enable MFX interrupt */
  HAL_NVIC_EnableIRQ(MFX_INT_EXTI_IRQn);
}


bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
	int16_t x, y;
	//static strType_XPT2046_Coordinate cinfo={-1,-1,-1,-1};
	
	if(MfxItOccurred == SET)
    {
        Mfx_Event();
		if(TouchEvent != 0){
			BSP_TS_GetState(&TS_State);
			if(TS_State.touchDetected){
				data->state = LV_INDEV_STATE_PR;
				x = TS_State.touchX[0];
				y = TS_State.touchY[0];	
			} else {
				x = -1;y = -1;
				data->state = LV_INDEV_STATE_REL;
			}
			TouchEvent = 0;
		}
	}
	 else {
		x = -1;y = -1;
		data->state = LV_INDEV_STATE_REL;
	}
//    /*Save the state and save the pressed coordinate*/
//	if(XPT2046_TouchDetect() == TOUCH_PRESSED){
//		data->state = LV_INDEV_STATE_PR;
//		XPT2046_Get_TouchedPoint(&cinfo,strXPT2046_TouchPara);
//		cinfo.pre_x = cinfo.x; cinfo.pre_y = cinfo.y;
//	} else {
//		data->state = LV_INDEV_STATE_REL;
//		cinfo.x = -1;
//		cinfo.y = -1; 
//		cinfo.pre_x = -1;
//		cinfo.pre_y = -1;
//	}

//    /*Set the coordinates (if released use the last pressed coordinates)*/
    data->point.x = x;
    data->point.y = y;

    return false; /*Return `false` because we are not buffering and no more data to read*/
}

typedef  FIL pc_file_t;
static lv_fs_res_t pcfs_open(lv_fs_drv_t * drv, void * file_p, const char * fn, lv_fs_mode_t mode);
static lv_fs_res_t pcfs_close(lv_fs_drv_t * drv, void * file_p);
static lv_fs_res_t pcfs_read(lv_fs_drv_t * drv, void * file_p, void * buf, uint32_t btr, uint32_t * br);
static lv_fs_res_t pcfs_seek(lv_fs_drv_t * drv, void * file_p, uint32_t pos);
static lv_fs_res_t pcfs_tell(lv_fs_drv_t * drv, void * file_p, uint32_t * pos_p);

extern int modules_paths_count;
extern char* modules_paths[];

FIL lvgl_file;

static lv_fs_res_t pcfs_open(lv_fs_drv_t * drv, void * file_p, const char * fn, lv_fs_mode_t mode)
{
    (void) drv; /*Unused*/

	char buf[128];
	sprintf(buf,  "0:/%s", fn);
	FRESULT res = f_open(&lvgl_file, buf, FA_READ | FA_OPEN_EXISTING);
    if(res != FR_OK) return LV_FS_RES_UNKNOWN;
    else {
		pc_file_t * fp = file_p;        /*Just avoid the confusing casings*/
        *fp = lvgl_file;
    }

    return LV_FS_RES_OK;
}


/**
 * Close an opened file
 * @param drv pointer to the current driver
 * @param file_p pointer to a FILE* variable. (opened with lv_ufs_open)
 * @return LV_FS_RES_OK: no error, the file is read
 *         any error from lv__fs_res_t enum
 */
static lv_fs_res_t pcfs_close(lv_fs_drv_t * drv, void * file_p)
{
    (void) drv; /*Unused*/
    f_close(&lvgl_file);
    return LV_FS_RES_OK;
}

/**
 * Read data from an opened file
 * @param drv pointer to the current driver
 * @param file_p pointer to a FILE variable.
 * @param buf pointer to a memory block where to store the read data
 * @param btr number of Bytes To Read
 * @param br the real number of read bytes (Byte Read)
 * @return LV_FS_RES_OK: no error, the file is read
 *         any error from lv__fs_res_t enum
 */
static lv_fs_res_t pcfs_read(lv_fs_drv_t * drv, void * file_p, void * buf, uint32_t btr, uint32_t * br)
{
    (void) drv; /*Unused*/
    f_read(&lvgl_file, buf, btr, br);
    return LV_FS_RES_OK;
}

/**
 * Set the read write pointer. Also expand the file size if necessary.
 * @param drv pointer to the current driver
 * @param file_p pointer to a FILE* variable. (opened with lv_ufs_open )
 * @param pos the new position of read write pointer
 * @return LV_FS_RES_OK: no error, the file is read
 *         any error from lv__fs_res_t enum
 */
static lv_fs_res_t pcfs_seek(lv_fs_drv_t * drv, void * file_p, uint32_t pos)
{
    (void) drv; /*Unused*/	
	f_lseek(&lvgl_file, pos);
    return LV_FS_RES_OK;
}

/**
 * Give the position of the read write pointer
 * @param drv pointer to the current driver
 * @param file_p pointer to a FILE* variable.
 * @param pos_p pointer to to store the result
 * @return LV_FS_RES_OK: no error, the file is read
 *         any error from lv__fs_res_t enum
 */
static lv_fs_res_t pcfs_tell(lv_fs_drv_t * drv, void * file_p, uint32_t * pos_p)
{
    (void) drv; /*Unused*/
    *pos_p = (uint32_t)f_tell(&lvgl_file);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_size_cb(lv_fs_drv_t *drv, void *file_p, uint32_t *size_p){
	pc_file_t * fp = file_p;
	*size_p = f_size(fp);
	return LV_FS_RES_OK;
}

void lvgl_image_driver_init(){
    lv_fs_drv_t pcfs_drv;                         /*A driver descriptor*/
    memset(&pcfs_drv, 0, sizeof(lv_fs_drv_t));    /*Initialization*/

    pcfs_drv.file_size = sizeof(pc_file_t);       /*Set up fields...*/
    pcfs_drv.letter = 'P';
    pcfs_drv.open_cb = pcfs_open;
    pcfs_drv.close_cb = pcfs_close;
    pcfs_drv.read_cb = pcfs_read;
    pcfs_drv.seek_cb = pcfs_seek;
    pcfs_drv.tell_cb = pcfs_tell;
	pcfs_drv.size_cb = pcfs_size_cb;
    lv_fs_drv_register(&pcfs_drv);
}

extern void lv_demo_widgets(void);

void lvgl_main()
{
	lv_init();
	lv_png_init();
	
	buf_1 = evm_malloc(LV_HOR_RES_MAX * LV_VER_RES_MAX);
	buf_2 = evm_malloc(LV_HOR_RES_MAX * LV_VER_RES_MAX);
    lv_disp_drv_t disp_drv;
    lv_indev_drv_t indev_drv;
    lv_disp_buf_init(&disp_buf, buf_1, buf_2, LV_HOR_RES_MAX * LV_VER_RES_MAX / 2);
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

    lv_indev_drv_init(&indev_drv);             /*Descriptor of a input device driver*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
    indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
    lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/

    lvgl_image_driver_init();
}
