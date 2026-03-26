#include "main.h"

u8 key=0,mode;

// 新增的全局变量
u8 gps_count = 0;      // 记录当前 GPS 点数量 (0~4)
u8 waiting_ack = 0;    // 防连发状态锁 (0:空闲, 1:正在等待Jetson回传)

int main(void)																																																																																																																																																																																																
{
	//初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级 初始化
	delay_init(168);		   //初始化延时函数
//	LED_Init();				    //初始化LED端口
//	BEEP_Init();
	KEY_Init();
	OLED_Init();
	EC11_Init();
	POWER_Init();
	ROCKER_Init();
	//MPU_UART1_init(115200);
	NRF24L01_Init();
	TIM6_init(10-1,8400-1); //0.01s定时中断
	TIM7_init(100-1,8400-1); //0.01s定时中断
    OLED_ShowString(0, 0, "GPS: 0", 16, 1);// 初始化时在屏幕上显示 GPS: 0
    OLED_Refresh();
	//主程序
	while(1){
		key_value = KEY_Scan(0);
		show_voltage(1);
		show_rocker_xy(20,20,50,20);
		//show_angle();
		if(0!=key_value){
			OLED_ShowNum(80,50,key_value,2,8,1);		
		}
		OLED_Refresh();
		delay_ms(10);
		// ========================================================
        // ★ 核心控制逻辑：PE6 检测与防连发发送锁
        // ========================================================
        // 1. 如果按下按键，触发锁存
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) == 0) {
            if (gps_count < 4 && waiting_ack == 0) {
                waiting_ack = 1; // 上锁：表示“我已经发起了请求，正等车体回复”
            }
        }

        // 2. 数据打包逻辑 (保证在收到应答前，一直发送 0x23 保证不丢包)
		tmp_buf[0]=32;             
		tmp_buf[1]=EC11_Num1;    
		tmp_buf[2]=adcx1;        
		tmp_buf[3]=adcy1;
		tmp_buf[4]=adcx2;
		tmp_buf[5]=adcy2;
        
        if (waiting_ack == 1) {
            tmp_buf[6] = 0x23; // 如果在等待应答，强制发送 RECORD_GPS (0x23)
        } else {
            tmp_buf[6] = key_value; // 否则正常发送矩阵键盘的值
        }

        // 3. 发射射频数据包
        if (NRF24L01_TxPacket(tmp_buf) == TX_OK) {
            
            // 发送成功后，立即检查 NRF24L01 的 FIFO，看车体有没有把 Jetson 的应答挂载送回来
            uint8_t fifo_status = NRF24L01_Read_Reg(NRF_READ_REG + NRF_FIFO_STATUS);
            
            // bit0 如果为 0，说明 RX FIFO 里面有数据！
            if ((fifo_status & 0x01) == 0) {
                uint8_t ack_data[32] = {0};
                
                // 把应答数据从硬件缓冲区读出来 (0x61 = RD_RX_PLOAD)
                NRF24L01_Read_Buf(RD_RX_PLOAD, ack_data, 1); 
                
                // 如果车体真的传回了 0x01 (代表 Jetson 记录成功)，且手柄在等这个包
                if (ack_data[0] == 0x01 && waiting_ack == 1) {
                    gps_count++;       // 终于完成！累加计数
                    waiting_ack = 0;   // 解锁，允许下一次按键
                    tmp_buf[6] = 0x00; // 撤销指令
                    
                    // 屏幕刷新最新累加值
                    char disp_buf[16];
                    sprintf(disp_buf, "GPS: %d", gps_count);
                    OLED_ShowString(0, 0, disp_buf, 16, 1);
                }
            }
        }
	}
	return 0;
}		
