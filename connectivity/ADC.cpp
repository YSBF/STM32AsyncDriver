

#include "stm32f4xx_adc.h"



static void ADC1_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  //打开DMA1的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 ,ENABLE);
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //模拟输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}


#define ADC1_DR_Address    ((u32)0x40012400+0x4c)  //外设地址
__IO uint16_t ADC_ConvertedValue[2];  //内存数组



static void ADC1_Mode_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;

  //---------------ADC的DMA配置--------------------
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;  //ADC1地址---代表ADC1保存转换值的寄存器
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&ADC_ConvertedValue;  //内存地址---用来保存DMA传输过来的ADC转换值----后面直接使用的变量地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  //外设为数据源
  DMA_InitStructure.DMA_BufferSize = 2;  //传输总数据---2通道需要传输2个数据
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址固定
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址自增---总体表示始终从外设ADC1地址处取值---依次保存到连续的两个内存变量中---
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;   //外设传输数据单元---半字16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //内存传输数据单元---半字16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //循环模式---2个数据依次循环接收从外设ADC1传输过来的ADC值---
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;  //高优先级
  DMA_Init(DMA2_Stream4, &DMA_InitStructure);

  DMA_Cmd(DMA2_Stream4, ENABLE);  //再次打开DMA1

  //------------ADC模式配置------------------------
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //独立模式----还有很多模式---这个比较常见
  ADC_InitStructure.ADC_ScanConvMode = ENABLE ;   //扫描模式---采集多通道使用----本程序采集2通道---所以扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //连续转换模式---不难理解---就是不停地采集---一次接一次
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //不使用外部触发转换---触发分为外部触发---比如中断与定时器。软件触发---后面有专用函数
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //采集的数据右对齐---方便计算
  ADC_InitStructure.ADC_NbrOfChannel = 2;  //总共需要转换的通道个数---这里2个
  ADC_Init(ADC1, &ADC_InitStructure);

  RCC_ADCCLKConfig(RCC_PCLK2_Div8);  //配置ADC转换时钟---PCLK2的8分频
  //下面这个函数比较重要----配置ADC的通道与采样周期---前面说的PC0与PC1对应的ADC通道分别是--10与11。采集周期也有几种。
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_55Cycles5);

  ADC_DMACmd(ADC1, ENABLE);  //打开DMA1的ADC1
  ADC_Cmd(ADC1, ENABLE);  //打开ADC1

  ADC_ResetCalibration(ADC1);  //复位校准寄存器
  while(ADC_GetResetCalibrationStatus(ADC1));  //等待校准寄存器复位完成

  ADC_StartCalibration(ADC1);  //ADC校准
  while(ADC_GetCalibrationStatus(ADC1));  //校准完成

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);  //软件触发转换
}




extern __IO uint16_t ADC_ConvertedValue[2];  //声明外部变量
uint16_t My_ADC[2];  //求平均值

int main(void)
{
  u8 i,led=0x01;



  ADC1_GPIO_Config();
  ADC1_Mode_Config();


  while (1)
  {
    My_ADC[0]=0;
    My_ADC[1]=0;


    for(i=0;i<10;i++)
    {
      My_ADC[0]+=ADC_ConvertedValue[0];
      My_ADC[1]+=ADC_ConvertedValue[1];
    }
    My_ADC[0]=My_ADC[0]/10;   //采集10次求平均值
    My_ADC[1]=My_ADC[1]/10;

    ADC_ConvertedValueLocal =(float) My_ADC[0]/4096*3.3;   //转换为电压值

    printf("\r\n The current AD---0 value = 0x%04X \n", My_ADC[0]);
    printf("The current AD---0 value = %f V \n",ADC_ConvertedValueLocal);

    ADC_ConvertedValueLocal =(float) My_ADC[1]/4096*3.3;

    printf("The current AD---1 value = 0x%04X \n", My_ADC[1]);
    printf("The current AD---1 value = %f V \n",ADC_ConvertedValueLocal);
  }
}

