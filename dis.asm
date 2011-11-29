
main.elf:     file format elf32-littlearm


Disassembly of section .text:

08000000 <__cs3_interrupt_vector>:
 8000000:	00 50 00 20 4d 76 00 08 5d 76 00 08 5d 76 00 08     .P. Mv..]v..]v..
 8000010:	5d 76 00 08 5d 76 00 08 5d 76 00 08 5d 76 00 08     ]v..]v..]v..]v..
 8000020:	5d 76 00 08 5d 76 00 08 5d 76 00 08 5d 76 00 08     ]v..]v..]v..]v..
 8000030:	5d 76 00 08 5d 76 00 08 5d 76 00 08 5d 76 00 08     ]v..]v..]v..]v..
 8000040:	21 3b 00 08 21 3b 00 08 21 3b 00 08 21 3b 00 08     !;..!;..!;..!;..
 8000050:	21 3b 00 08 21 3b 00 08 c9 3a 00 08 21 3b 00 08     !;..!;...:..!;..
 8000060:	e9 3a 00 08 21 3b 00 08 21 3b 00 08 5d 76 00 08     .:..!;..!;..]v..
 8000070:	5d 76 00 08 5d 76 00 08 5d 76 00 08 5d 76 00 08     ]v..]v..]v..]v..
 8000080:	5d 76 00 08 5d 76 00 08 5d 76 00 08 09 3b 00 08     ]v..]v..]v...;..
 8000090:	15 3b 00 08 5d 76 00 08 5d 76 00 08 21 3b 00 08     .;..]v..]v..!;..
 80000a0:	21 3b 00 08 21 3b 00 08 21 3b 00 08 21 3b 00 08     !;..!;..!;..!;..
 80000b0:	21 3b 00 08 21 3b 00 08 21 3b 00 08 21 3b 00 08     !;..!;..!;..!;..
 80000c0:	21 3b 00 08 21 3b 00 08 21 3b 00 08 21 3b 00 08     !;..!;..!;..!;..
 80000d0:	21 3b 00 08 21 3b 00 08 21 3b 00 08 21 3b 00 08     !;..!;..!;..!;..
 80000e0:	21 3b 00 08 21 3b 00 08 21 3b 00 08                 !;..!;..!;..

080000ec <__do_global_dtors_aux>:
 80000ec:	f240 1370 	movw	r3, #368	; 0x170
 80000f0:	f2c2 0300 	movt	r3, #8192	; 0x2000
 80000f4:	781a      	ldrb	r2, [r3, #0]
 80000f6:	b90a      	cbnz	r2, 80000fc <__do_global_dtors_aux+0x10>
 80000f8:	2001      	movs	r0, #1
 80000fa:	7018      	strb	r0, [r3, #0]
 80000fc:	4770      	bx	lr
 80000fe:	bf00      	nop

08000100 <frame_dummy>:
 8000100:	f240 0000 	movw	r0, #0
 8000104:	f2c2 0000 	movt	r0, #8192	; 0x2000
 8000108:	b508      	push	{r3, lr}
 800010a:	6803      	ldr	r3, [r0, #0]
 800010c:	b12b      	cbz	r3, 800011a <frame_dummy+0x1a>
 800010e:	f240 0300 	movw	r3, #0
 8000112:	f2c0 0300 	movt	r3, #0
 8000116:	b103      	cbz	r3, 800011a <frame_dummy+0x1a>
 8000118:	4798      	blx	r3
 800011a:	bd08      	pop	{r3, pc}
 800011c:	0000      	movs	r0, r0
	...

08000120 <calibrate_sensor>:

#define DIFF_GAIN 1.0/(float)113.23			/*pressure sensor 23mv/10v at 5psi, +60.1x inst amp gain, +12bit adc*/

static float pressure_offset;			//zero offset - calibrated at power on

void calibrate_sensor(void) {
 8000120:	b5b0      	push	{r4, r5, r7, lr}
 8000122:	b082      	sub	sp, #8
 8000124:	af00      	add	r7, sp, #0
	float pressoff=0;
 8000126:	4b1d      	ldr	r3, [pc, #116]	; (800019c <calibrate_sensor+0x7c>)
 8000128:	607b      	str	r3, [r7, #4]
	for(uint8_t n=0;n<100;n++)
 800012a:	f04f 0300 	mov.w	r3, #0
 800012e:	70fb      	strb	r3, [r7, #3]
 8000130:	e028      	b.n	8000184 <calibrate_sensor+0x64>
		pressoff+=conv_adc_diff()/100.0;
 8000132:	6878      	ldr	r0, [r7, #4]
 8000134:	f006 fca0 	bl	8006a78 <__aeabi_f2d>
 8000138:	4604      	mov	r4, r0
 800013a:	460d      	mov	r5, r1
 800013c:	f000 f830 	bl	80001a0 <conv_adc_diff>
 8000140:	4603      	mov	r3, r0
 8000142:	4618      	mov	r0, r3
 8000144:	f006 fc98 	bl	8006a78 <__aeabi_f2d>
 8000148:	4602      	mov	r2, r0
 800014a:	460b      	mov	r3, r1
 800014c:	4610      	mov	r0, r2
 800014e:	4619      	mov	r1, r3
 8000150:	f04f 0200 	mov.w	r2, #0
 8000154:	f04f 4380 	mov.w	r3, #1073741824	; 0x40000000
 8000158:	f503 03b2 	add.w	r3, r3, #5832704	; 0x590000
 800015c:	f006 fe0a 	bl	8006d74 <__aeabi_ddiv>
 8000160:	4602      	mov	r2, r0
 8000162:	460b      	mov	r3, r1
 8000164:	4620      	mov	r0, r4
 8000166:	4629      	mov	r1, r5
 8000168:	f006 fb28 	bl	80067bc <__adddf3>
 800016c:	4602      	mov	r2, r0
 800016e:	460b      	mov	r3, r1
 8000170:	4610      	mov	r0, r2
 8000172:	4619      	mov	r1, r3
 8000174:	f006 fee6 	bl	8006f44 <__aeabi_d2f>
 8000178:	4603      	mov	r3, r0
 800017a:	607b      	str	r3, [r7, #4]

static float pressure_offset;			//zero offset - calibrated at power on

void calibrate_sensor(void) {
	float pressoff=0;
	for(uint8_t n=0;n<100;n++)
 800017c:	78fb      	ldrb	r3, [r7, #3]
 800017e:	f103 0301 	add.w	r3, r3, #1
 8000182:	70fb      	strb	r3, [r7, #3]
 8000184:	78fb      	ldrb	r3, [r7, #3]
 8000186:	2b63      	cmp	r3, #99	; 0x63
 8000188:	d9d3      	bls.n	8000132 <calibrate_sensor+0x12>
		pressoff+=conv_adc_diff()/100.0;
	pressure_offset=pressoff;
 800018a:	4b03      	ldr	r3, [pc, #12]	; (8000198 <calibrate_sensor+0x78>)
 800018c:	687a      	ldr	r2, [r7, #4]
 800018e:	601a      	str	r2, [r3, #0]
}
 8000190:	f107 0708 	add.w	r7, r7, #8
 8000194:	46bd      	mov	sp, r7
 8000196:	bdb0      	pop	{r4, r5, r7, pc}
 8000198:	20000174 	.word	0x20000174
 800019c:	00000000 	.word	0x00000000

080001a0 <conv_adc_diff>:


float conv_adc_diff(void) {
 80001a0:	b5b0      	push	{r4, r5, r7, lr}
 80001a2:	b082      	sub	sp, #8
 80001a4:	af00      	add	r7, sp, #0
	uint16_t p=readADC1(1);
 80001a6:	f04f 0001 	mov.w	r0, #1
 80001aa:	f003 fb1d 	bl	80037e8 <readADC1>
 80001ae:	4603      	mov	r3, r0
 80001b0:	80fb      	strh	r3, [r7, #6]
	return 	(DIFF_GAIN*(float)p)-pressure_offset;
 80001b2:	88fb      	ldrh	r3, [r7, #6]
 80001b4:	4618      	mov	r0, r3
 80001b6:	f006 ffcb 	bl	8007150 <__aeabi_ui2f>
 80001ba:	4603      	mov	r3, r0
 80001bc:	4618      	mov	r0, r3
 80001be:	f006 fc5b 	bl	8006a78 <__aeabi_f2d>
 80001c2:	4602      	mov	r2, r0
 80001c4:	460b      	mov	r3, r1
 80001c6:	4610      	mov	r0, r2
 80001c8:	4619      	mov	r1, r3
 80001ca:	a311      	add	r3, pc, #68	; (adr r3, 8000210 <conv_adc_diff+0x70>)
 80001cc:	e9d3 2300 	ldrd	r2, r3, [r3]
 80001d0:	f006 fca6 	bl	8006b20 <__aeabi_dmul>
 80001d4:	4602      	mov	r2, r0
 80001d6:	460b      	mov	r3, r1
 80001d8:	4614      	mov	r4, r2
 80001da:	461d      	mov	r5, r3
 80001dc:	4b0e      	ldr	r3, [pc, #56]	; (8000218 <conv_adc_diff+0x78>)
 80001de:	681b      	ldr	r3, [r3, #0]
 80001e0:	4618      	mov	r0, r3
 80001e2:	f006 fc49 	bl	8006a78 <__aeabi_f2d>
 80001e6:	4602      	mov	r2, r0
 80001e8:	460b      	mov	r3, r1
 80001ea:	4620      	mov	r0, r4
 80001ec:	4629      	mov	r1, r5
 80001ee:	f006 fae3 	bl	80067b8 <__aeabi_dsub>
 80001f2:	4602      	mov	r2, r0
 80001f4:	460b      	mov	r3, r1
 80001f6:	4610      	mov	r0, r2
 80001f8:	4619      	mov	r1, r3
 80001fa:	f006 fea3 	bl	8006f44 <__aeabi_d2f>
 80001fe:	4603      	mov	r3, r0
}
 8000200:	4618      	mov	r0, r3
 8000202:	f107 0708 	add.w	r7, r7, #8
 8000206:	46bd      	mov	sp, r7
 8000208:	bdb0      	pop	{r4, r5, r7, pc}
 800020a:	bf00      	nop
 800020c:	f3af 8000 	nop.w
 8000210:	ccd68c5d 	.word	0xccd68c5d
 8000214:	3f82164a 	.word	0x3f82164a
 8000218:	20000174 	.word	0x20000174
 800021c:	f3af 8000 	nop.w

08000220 <Set_System>:
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
 8000220:	b580      	push	{r7, lr}
 8000222:	af00      	add	r7, sp, #0
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);  
#endif /* USB_USE_EXTERNAL_PULLUP */  
  
  /* MAL configuration */
  MAL_Config();
 8000224:	f000 f918 	bl	8000458 <MAL_Config>
}
 8000228:	bd80      	pop	{r7, pc}
 800022a:	bf00      	nop

0800022c <Set_USBClock>:
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
 800022c:	b580      	push	{r7, lr}
 800022e:	af00      	add	r7, sp, #0

  /* Enable the USB clock */ 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE) ;
#else
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
 8000230:	f04f 0000 	mov.w	r0, #0
 8000234:	f004 fb3c 	bl	80048b0 <RCC_USBCLKConfig>
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
 8000238:	f44f 0000 	mov.w	r0, #8388608	; 0x800000
 800023c:	f04f 0101 	mov.w	r1, #1
 8000240:	f004 fbd4 	bl	80049ec <RCC_APB1PeriphClockCmd>
#endif /* STM32L1XX_MD */
}
 8000244:	bd80      	pop	{r7, pc}
 8000246:	bf00      	nop

08000248 <Enter_LowPowerMode>:
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
 8000248:	b480      	push	{r7}
 800024a:	af00      	add	r7, sp, #0
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
 800024c:	4b03      	ldr	r3, [pc, #12]	; (800025c <Enter_LowPowerMode+0x14>)
 800024e:	f04f 0203 	mov.w	r2, #3
 8000252:	601a      	str	r2, [r3, #0]
}
 8000254:	46bd      	mov	sp, r7
 8000256:	bc80      	pop	{r7}
 8000258:	4770      	bx	lr
 800025a:	bf00      	nop
 800025c:	200001ac 	.word	0x200001ac

08000260 <Leave_LowPowerMode>:
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
 8000260:	b480      	push	{r7}
 8000262:	b083      	sub	sp, #12
 8000264:	af00      	add	r7, sp, #0
  DEVICE_INFO *pInfo = &Device_Info;
 8000266:	4b0a      	ldr	r3, [pc, #40]	; (8000290 <Leave_LowPowerMode+0x30>)
 8000268:	607b      	str	r3, [r7, #4]

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
 800026a:	687b      	ldr	r3, [r7, #4]
 800026c:	7a9b      	ldrb	r3, [r3, #10]
 800026e:	2b00      	cmp	r3, #0
 8000270:	d004      	beq.n	800027c <Leave_LowPowerMode+0x1c>
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
 8000272:	4b08      	ldr	r3, [pc, #32]	; (8000294 <Leave_LowPowerMode+0x34>)
 8000274:	f04f 0205 	mov.w	r2, #5
 8000278:	601a      	str	r2, [r3, #0]
 800027a:	e003      	b.n	8000284 <Leave_LowPowerMode+0x24>
  }
  else
  {
    bDeviceState = ATTACHED;
 800027c:	4b05      	ldr	r3, [pc, #20]	; (8000294 <Leave_LowPowerMode+0x34>)
 800027e:	f04f 0201 	mov.w	r2, #1
 8000282:	601a      	str	r2, [r3, #0]
  }

}
 8000284:	f107 070c 	add.w	r7, r7, #12
 8000288:	46bd      	mov	sp, r7
 800028a:	bc80      	pop	{r7}
 800028c:	4770      	bx	lr
 800028e:	bf00      	nop
 8000290:	200004ec 	.word	0x200004ec
 8000294:	200001ac 	.word	0x200001ac

08000298 <USB_Interrupts_Config>:
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
 8000298:	b580      	push	{r7, lr}
 800029a:	b082      	sub	sp, #8
 800029c:	af00      	add	r7, sp, #0
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
 800029e:	f44f 60c0 	mov.w	r0, #1536	; 0x600
 80002a2:	f003 fe29 	bl	8003ef8 <NVIC_PriorityGroupConfig>
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#else
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
 80002a6:	f04f 0314 	mov.w	r3, #20
 80002aa:	713b      	strb	r3, [r7, #4]
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
 80002ac:	f04f 0301 	mov.w	r3, #1
 80002b0:	717b      	strb	r3, [r7, #5]
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 80002b2:	f04f 0301 	mov.w	r3, #1
 80002b6:	71bb      	strb	r3, [r7, #6]
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 80002b8:	f04f 0301 	mov.w	r3, #1
 80002bc:	71fb      	strb	r3, [r7, #7]
  NVIC_Init(&NVIC_InitStructure);
 80002be:	f107 0304 	add.w	r3, r7, #4
 80002c2:	4618      	mov	r0, r3
 80002c4:	f003 fe22 	bl	8003f0c <NVIC_Init>

  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
 80002c8:	f04f 0313 	mov.w	r3, #19
 80002cc:	713b      	strb	r3, [r7, #4]
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
 80002ce:	f04f 0301 	mov.w	r3, #1
 80002d2:	717b      	strb	r3, [r7, #5]
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 80002d4:	f04f 0300 	mov.w	r3, #0
 80002d8:	71bb      	strb	r3, [r7, #6]
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 80002da:	f04f 0301 	mov.w	r3, #1
 80002de:	71fb      	strb	r3, [r7, #7]
  NVIC_Init(&NVIC_InitStructure);
 80002e0:	f107 0304 	add.w	r3, r7, #4
 80002e4:	4618      	mov	r0, r3
 80002e6:	f003 fe11 	bl	8003f0c <NVIC_Init>
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
#endif /* STM32L1XX_MD */
  
}
 80002ea:	f107 0708 	add.w	r7, r7, #8
 80002ee:	46bd      	mov	sp, r7
 80002f0:	bd80      	pop	{r7, pc}
 80002f2:	bf00      	nop

080002f4 <Led_RW_ON>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Led_RW_ON(void)
{
 80002f4:	b580      	push	{r7, lr}
 80002f6:	af00      	add	r7, sp, #0
  GREEN_LED_ON;
 80002f8:	4803      	ldr	r0, [pc, #12]	; (8000308 <Led_RW_ON+0x14>)
 80002fa:	f44f 5180 	mov.w	r1, #4096	; 0x1000
 80002fe:	f04f 0201 	mov.w	r2, #1
 8000302:	f004 f964 	bl	80045ce <GPIO_WriteBit>
}
 8000306:	bd80      	pop	{r7, pc}
 8000308:	40010c00 	.word	0x40010c00

0800030c <Led_RW_OFF>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Led_RW_OFF(void)
{
 800030c:	b580      	push	{r7, lr}
 800030e:	af00      	add	r7, sp, #0
  GREEN_LED_OFF;
 8000310:	4803      	ldr	r0, [pc, #12]	; (8000320 <Led_RW_OFF+0x14>)
 8000312:	f44f 5180 	mov.w	r1, #4096	; 0x1000
 8000316:	f04f 0200 	mov.w	r2, #0
 800031a:	f004 f958 	bl	80045ce <GPIO_WriteBit>
}
 800031e:	bd80      	pop	{r7, pc}
 8000320:	40010c00 	.word	0x40010c00

08000324 <USB_Configured_LED>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Configured_LED(void)
{
 8000324:	b580      	push	{r7, lr}
 8000326:	af00      	add	r7, sp, #0
  RED_LED_ON;
 8000328:	4803      	ldr	r0, [pc, #12]	; (8000338 <USB_Configured_LED+0x14>)
 800032a:	f44f 6100 	mov.w	r1, #2048	; 0x800
 800032e:	f04f 0201 	mov.w	r2, #1
 8000332:	f004 f94c 	bl	80045ce <GPIO_WriteBit>
}
 8000336:	bd80      	pop	{r7, pc}
 8000338:	40010c00 	.word	0x40010c00

0800033c <USB_NotConfigured_LED>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_NotConfigured_LED(void)
{
 800033c:	b580      	push	{r7, lr}
 800033e:	af00      	add	r7, sp, #0
  RED_LED_OFF;
 8000340:	4803      	ldr	r0, [pc, #12]	; (8000350 <USB_NotConfigured_LED+0x14>)
 8000342:	f44f 6100 	mov.w	r1, #2048	; 0x800
 8000346:	f04f 0200 	mov.w	r2, #0
 800034a:	f004 f940 	bl	80045ce <GPIO_WriteBit>
}
 800034e:	bd80      	pop	{r7, pc}
 8000350:	40010c00 	.word	0x40010c00

08000354 <USB_Cable_Config>:
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
 8000354:	b480      	push	{r7}
 8000356:	b083      	sub	sp, #12
 8000358:	af00      	add	r7, sp, #0
 800035a:	4603      	mov	r3, r0
 800035c:	71fb      	strb	r3, [r7, #7]
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* STM32L1XX_MD */
}
 800035e:	f107 070c 	add.w	r7, r7, #12
 8000362:	46bd      	mov	sp, r7
 8000364:	bc80      	pop	{r7}
 8000366:	4770      	bx	lr

08000368 <Get_SerialNum>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
 8000368:	b580      	push	{r7, lr}
 800036a:	b084      	sub	sp, #16
 800036c:	af00      	add	r7, sp, #0
#ifdef STM32L1XX_MD
  Device_Serial0 = *(uint32_t*)(0x1FF80050);
  Device_Serial1 = *(uint32_t*)(0x1FF80054);
  Device_Serial2 = *(uint32_t*)(0x1FF80064);
#else  
  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
 800036e:	4b10      	ldr	r3, [pc, #64]	; (80003b0 <Get_SerialNum+0x48>)
 8000370:	681b      	ldr	r3, [r3, #0]
 8000372:	60fb      	str	r3, [r7, #12]
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
 8000374:	4b0f      	ldr	r3, [pc, #60]	; (80003b4 <Get_SerialNum+0x4c>)
 8000376:	681b      	ldr	r3, [r3, #0]
 8000378:	60bb      	str	r3, [r7, #8]
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);
 800037a:	4b0f      	ldr	r3, [pc, #60]	; (80003b8 <Get_SerialNum+0x50>)
 800037c:	681b      	ldr	r3, [r3, #0]
 800037e:	607b      	str	r3, [r7, #4]
#endif /* STM32L1XX_MD */

  Device_Serial0 += Device_Serial2;
 8000380:	68fa      	ldr	r2, [r7, #12]
 8000382:	687b      	ldr	r3, [r7, #4]
 8000384:	18d3      	adds	r3, r2, r3
 8000386:	60fb      	str	r3, [r7, #12]

  if (Device_Serial0 != 0)
 8000388:	68fb      	ldr	r3, [r7, #12]
 800038a:	2b00      	cmp	r3, #0
 800038c:	d00b      	beq.n	80003a6 <Get_SerialNum+0x3e>
  {
    IntToUnicode (Device_Serial0, &MASS_StringSerial[2] , 8);
 800038e:	68f8      	ldr	r0, [r7, #12]
 8000390:	490a      	ldr	r1, [pc, #40]	; (80003bc <Get_SerialNum+0x54>)
 8000392:	f04f 0208 	mov.w	r2, #8
 8000396:	f000 f815 	bl	80003c4 <IntToUnicode>
    IntToUnicode (Device_Serial1, &MASS_StringSerial[18], 4);
 800039a:	68b8      	ldr	r0, [r7, #8]
 800039c:	4908      	ldr	r1, [pc, #32]	; (80003c0 <Get_SerialNum+0x58>)
 800039e:	f04f 0204 	mov.w	r2, #4
 80003a2:	f000 f80f 	bl	80003c4 <IntToUnicode>
  }
}
 80003a6:	f107 0710 	add.w	r7, r7, #16
 80003aa:	46bd      	mov	sp, r7
 80003ac:	bd80      	pop	{r7, pc}
 80003ae:	bf00      	nop
 80003b0:	1ffff7e8 	.word	0x1ffff7e8
 80003b4:	1ffff7ec 	.word	0x1ffff7ec
 80003b8:	1ffff7f0 	.word	0x1ffff7f0
 80003bc:	2000007a 	.word	0x2000007a
 80003c0:	2000008a 	.word	0x2000008a

080003c4 <IntToUnicode>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
 80003c4:	b480      	push	{r7}
 80003c6:	b087      	sub	sp, #28
 80003c8:	af00      	add	r7, sp, #0
 80003ca:	60f8      	str	r0, [r7, #12]
 80003cc:	60b9      	str	r1, [r7, #8]
 80003ce:	4613      	mov	r3, r2
 80003d0:	71fb      	strb	r3, [r7, #7]
  uint8_t idx = 0;
 80003d2:	f04f 0300 	mov.w	r3, #0
 80003d6:	75fb      	strb	r3, [r7, #23]
  
  for( idx = 0 ; idx < len ; idx ++)
 80003d8:	f04f 0300 	mov.w	r3, #0
 80003dc:	75fb      	strb	r3, [r7, #23]
 80003de:	e031      	b.n	8000444 <IntToUnicode+0x80>
  {
    if( ((value >> 28)) < 0xA )
 80003e0:	68fb      	ldr	r3, [r7, #12]
 80003e2:	ea4f 7313 	mov.w	r3, r3, lsr #28
 80003e6:	2b09      	cmp	r3, #9
 80003e8:	d80d      	bhi.n	8000406 <IntToUnicode+0x42>
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
 80003ea:	7dfb      	ldrb	r3, [r7, #23]
 80003ec:	ea4f 0343 	mov.w	r3, r3, lsl #1
 80003f0:	68ba      	ldr	r2, [r7, #8]
 80003f2:	18d3      	adds	r3, r2, r3
 80003f4:	68fa      	ldr	r2, [r7, #12]
 80003f6:	ea4f 7212 	mov.w	r2, r2, lsr #28
 80003fa:	b2d2      	uxtb	r2, r2
 80003fc:	f102 0230 	add.w	r2, r2, #48	; 0x30
 8000400:	b2d2      	uxtb	r2, r2
 8000402:	701a      	strb	r2, [r3, #0]
 8000404:	e00c      	b.n	8000420 <IntToUnicode+0x5c>
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
 8000406:	7dfb      	ldrb	r3, [r7, #23]
 8000408:	ea4f 0343 	mov.w	r3, r3, lsl #1
 800040c:	68ba      	ldr	r2, [r7, #8]
 800040e:	18d3      	adds	r3, r2, r3
 8000410:	68fa      	ldr	r2, [r7, #12]
 8000412:	ea4f 7212 	mov.w	r2, r2, lsr #28
 8000416:	b2d2      	uxtb	r2, r2
 8000418:	f102 0237 	add.w	r2, r2, #55	; 0x37
 800041c:	b2d2      	uxtb	r2, r2
 800041e:	701a      	strb	r2, [r3, #0]
    }
    
    value = value << 4;
 8000420:	68fb      	ldr	r3, [r7, #12]
 8000422:	ea4f 1303 	mov.w	r3, r3, lsl #4
 8000426:	60fb      	str	r3, [r7, #12]
    
    pbuf[ 2* idx + 1] = 0;
 8000428:	7dfb      	ldrb	r3, [r7, #23]
 800042a:	ea4f 0343 	mov.w	r3, r3, lsl #1
 800042e:	f103 0301 	add.w	r3, r3, #1
 8000432:	68ba      	ldr	r2, [r7, #8]
 8000434:	18d3      	adds	r3, r2, r3
 8000436:	f04f 0200 	mov.w	r2, #0
 800043a:	701a      	strb	r2, [r3, #0]
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
 800043c:	7dfb      	ldrb	r3, [r7, #23]
 800043e:	f103 0301 	add.w	r3, r3, #1
 8000442:	75fb      	strb	r3, [r7, #23]
 8000444:	7dfa      	ldrb	r2, [r7, #23]
 8000446:	79fb      	ldrb	r3, [r7, #7]
 8000448:	429a      	cmp	r2, r3
 800044a:	d3c9      	bcc.n	80003e0 <IntToUnicode+0x1c>
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}
 800044c:	f107 071c 	add.w	r7, r7, #28
 8000450:	46bd      	mov	sp, r7
 8000452:	bc80      	pop	{r7}
 8000454:	4770      	bx	lr
 8000456:	bf00      	nop

08000458 <MAL_Config>:
* Description    : MAL_layer configuration
* Input          : None.
* Return         : None.
*******************************************************************************/
void MAL_Config(void)
{
 8000458:	b580      	push	{r7, lr}
 800045a:	af00      	add	r7, sp, #0
  MAL_Init(0);
 800045c:	f04f 0000 	mov.w	r0, #0
 8000460:	f000 f802 	bl	8000468 <MAL_Init>
#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
  /* Enable the FSMC Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
  MAL_Init(1);
#endif /* STM32F10X_HD | STM32F10X_XL */
}
 8000464:	bd80      	pop	{r7, pc}
 8000466:	bf00      	nop

08000468 <MAL_Init>:
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Init(uint8_t lun)
{
 8000468:	b580      	push	{r7, lr}
 800046a:	b084      	sub	sp, #16
 800046c:	af00      	add	r7, sp, #0
 800046e:	4603      	mov	r3, r0
 8000470:	71fb      	strb	r3, [r7, #7]
  uint16_t status = MAL_OK;
 8000472:	f04f 0300 	mov.w	r3, #0
 8000476:	81fb      	strh	r3, [r7, #14]

  switch (lun)
 8000478:	79fb      	ldrb	r3, [r7, #7]
 800047a:	2b00      	cmp	r3, #0
 800047c:	d107      	bne.n	800048e <MAL_Init+0x26>
  {
    case 0:
      Status = SD_Init();
 800047e:	f002 fb97 	bl	8002bb0 <SD_Init>
 8000482:	4603      	mov	r3, r0
 8000484:	461a      	mov	r2, r3
 8000486:	4b05      	ldr	r3, [pc, #20]	; (800049c <MAL_Init+0x34>)
 8000488:	601a      	str	r2, [r3, #0]
      break;
#endif
    default:
      return MAL_FAIL;
  }
  return status;
 800048a:	89fb      	ldrh	r3, [r7, #14]
 800048c:	e001      	b.n	8000492 <MAL_Init+0x2a>
    case 1:
      NAND_Init();
      break;
#endif
    default:
      return MAL_FAIL;
 800048e:	f04f 0301 	mov.w	r3, #1
  }
  return status;
}
 8000492:	4618      	mov	r0, r3
 8000494:	f107 0710 	add.w	r7, r7, #16
 8000498:	46bd      	mov	sp, r7
 800049a:	bd80      	pop	{r7, pc}
 800049c:	20000178 	.word	0x20000178

080004a0 <MAL_Write>:
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, uint32_t *Writebuff, uint16_t Transfer_Length)
{
 80004a0:	b580      	push	{r7, lr}
 80004a2:	b084      	sub	sp, #16
 80004a4:	af00      	add	r7, sp, #0
 80004a6:	60b9      	str	r1, [r7, #8]
 80004a8:	607a      	str	r2, [r7, #4]
 80004aa:	4602      	mov	r2, r0
 80004ac:	73fa      	strb	r2, [r7, #15]
 80004ae:	807b      	strh	r3, [r7, #2]

  switch (lun)
 80004b0:	7bfb      	ldrb	r3, [r7, #15]
 80004b2:	2b00      	cmp	r3, #0
 80004b4:	d10d      	bne.n	80004d2 <MAL_Write+0x32>
  {
    case 0:
      Status = SD_WriteBlock((uint8_t*)Writebuff, Memory_Offset, Transfer_Length);
 80004b6:	687a      	ldr	r2, [r7, #4]
 80004b8:	887b      	ldrh	r3, [r7, #2]
 80004ba:	4610      	mov	r0, r2
 80004bc:	68b9      	ldr	r1, [r7, #8]
 80004be:	461a      	mov	r2, r3
 80004c0:	f002 fca4 	bl	8002e0c <SD_WriteBlock>
 80004c4:	4603      	mov	r3, r0
 80004c6:	461a      	mov	r2, r3
 80004c8:	4b05      	ldr	r3, [pc, #20]	; (80004e0 <MAL_Write+0x40>)
 80004ca:	601a      	str	r2, [r3, #0]
      break;
#endif /* USE_STM3210E_EVAL */  
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
 80004cc:	f04f 0300 	mov.w	r3, #0
 80004d0:	e001      	b.n	80004d6 <MAL_Write+0x36>
    case 1:
      NAND_Write(Memory_Offset, Writebuff, Transfer_Length);
      break;
#endif /* USE_STM3210E_EVAL */  
    default:
      return MAL_FAIL;
 80004d2:	f04f 0301 	mov.w	r3, #1
  }
  return MAL_OK;
}
 80004d6:	4618      	mov	r0, r3
 80004d8:	f107 0710 	add.w	r7, r7, #16
 80004dc:	46bd      	mov	sp, r7
 80004de:	bd80      	pop	{r7, pc}
 80004e0:	20000178 	.word	0x20000178

080004e4 <MAL_Read>:
* Input          : None
* Output         : None
* Return         : Buffer pointer
*******************************************************************************/
uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, uint32_t *Readbuff, uint16_t Transfer_Length)
{
 80004e4:	b580      	push	{r7, lr}
 80004e6:	b084      	sub	sp, #16
 80004e8:	af00      	add	r7, sp, #0
 80004ea:	60b9      	str	r1, [r7, #8]
 80004ec:	607a      	str	r2, [r7, #4]
 80004ee:	4602      	mov	r2, r0
 80004f0:	73fa      	strb	r2, [r7, #15]
 80004f2:	807b      	strh	r3, [r7, #2]

  switch (lun)
 80004f4:	7bfb      	ldrb	r3, [r7, #15]
 80004f6:	2b00      	cmp	r3, #0
 80004f8:	d10d      	bne.n	8000516 <MAL_Read+0x32>
  {
    case 0:
      Status = SD_ReadBlock((uint8_t*)Readbuff, Memory_Offset, Transfer_Length);
 80004fa:	687a      	ldr	r2, [r7, #4]
 80004fc:	887b      	ldrh	r3, [r7, #2]
 80004fe:	4610      	mov	r0, r2
 8000500:	68b9      	ldr	r1, [r7, #8]
 8000502:	461a      	mov	r2, r3
 8000504:	f002 fbb8 	bl	8002c78 <SD_ReadBlock>
 8000508:	4603      	mov	r3, r0
 800050a:	461a      	mov	r2, r3
 800050c:	4b05      	ldr	r3, [pc, #20]	; (8000524 <MAL_Read+0x40>)
 800050e:	601a      	str	r2, [r3, #0]
      break;
#endif
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
 8000510:	f04f 0300 	mov.w	r3, #0
 8000514:	e001      	b.n	800051a <MAL_Read+0x36>
      NAND_Read(Memory_Offset, Readbuff, Transfer_Length);
      ;
      break;
#endif
    default:
      return MAL_FAIL;
 8000516:	f04f 0301 	mov.w	r3, #1
  }
  return MAL_OK;
}
 800051a:	4618      	mov	r0, r3
 800051c:	f107 0710 	add.w	r7, r7, #16
 8000520:	46bd      	mov	sp, r7
 8000522:	bd80      	pop	{r7, pc}
 8000524:	20000178 	.word	0x20000178

08000528 <MAL_GetStatus>:
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_GetStatus (uint8_t lun)
{
 8000528:	b580      	push	{r7, lr}
 800052a:	b090      	sub	sp, #64	; 0x40
 800052c:	af00      	add	r7, sp, #0
 800052e:	4603      	mov	r3, r0
 8000530:	71fb      	strb	r3, [r7, #7]
#ifdef USE_STM3210E_EVAL
  NAND_IDTypeDef NAND_ID;
  uint32_t DeviceSizeMul = 0, NumberOfBlocks = 0;
#else
  uint32_t temp_block_mul = 0;
 8000532:	f04f 0300 	mov.w	r3, #0
 8000536:	63fb      	str	r3, [r7, #60]	; 0x3c
  SD_CSD SD_csd;
  uint32_t DeviceSizeMul = 0;
 8000538:	f04f 0300 	mov.w	r3, #0
 800053c:	63bb      	str	r3, [r7, #56]	; 0x38
#endif /* USE_STM3210E_EVAL */


  if (lun == 0)
 800053e:	79fb      	ldrb	r3, [r7, #7]
 8000540:	2b00      	cmp	r3, #0
 8000542:	d142      	bne.n	80005ca <MAL_GetStatus+0xa2>
      {
        return MAL_FAIL;
      } 
     
#else
    SD_GetCSDRegister(&SD_csd);
 8000544:	f107 030c 	add.w	r3, r7, #12
 8000548:	4618      	mov	r0, r3
 800054a:	f002 fd33 	bl	8002fb4 <SD_GetCSDRegister>
    DeviceSizeMul = SD_csd.DeviceSizeMul + 2;
 800054e:	f897 3024 	ldrb.w	r3, [r7, #36]	; 0x24
 8000552:	b2db      	uxtb	r3, r3
 8000554:	f103 0302 	add.w	r3, r3, #2
 8000558:	63bb      	str	r3, [r7, #56]	; 0x38
    temp_block_mul = (1 << SD_csd.RdBlockLen)/ 512;
 800055a:	7d3b      	ldrb	r3, [r7, #20]
 800055c:	b2db      	uxtb	r3, r3
 800055e:	f04f 0201 	mov.w	r2, #1
 8000562:	fa02 f303 	lsl.w	r3, r2, r3
 8000566:	2b00      	cmp	r3, #0
 8000568:	da03      	bge.n	8000572 <MAL_GetStatus+0x4a>
 800056a:	f503 73fe 	add.w	r3, r3, #508	; 0x1fc
 800056e:	f103 0303 	add.w	r3, r3, #3
 8000572:	ea4f 2363 	mov.w	r3, r3, asr #9
 8000576:	63fb      	str	r3, [r7, #60]	; 0x3c
    Mass_Block_Count[0] = ((SD_csd.DeviceSize + 1) * (1 << (DeviceSizeMul))) * temp_block_mul;
 8000578:	69fb      	ldr	r3, [r7, #28]
 800057a:	f103 0201 	add.w	r2, r3, #1
 800057e:	6bbb      	ldr	r3, [r7, #56]	; 0x38
 8000580:	fa02 f303 	lsl.w	r3, r2, r3
 8000584:	6bfa      	ldr	r2, [r7, #60]	; 0x3c
 8000586:	fb02 f203 	mul.w	r2, r2, r3
 800058a:	4b17      	ldr	r3, [pc, #92]	; (80005e8 <MAL_GetStatus+0xc0>)
 800058c:	601a      	str	r2, [r3, #0]
    Mass_Block_Size[0] = 512;
 800058e:	4b17      	ldr	r3, [pc, #92]	; (80005ec <MAL_GetStatus+0xc4>)
 8000590:	f44f 7200 	mov.w	r2, #512	; 0x200
 8000594:	601a      	str	r2, [r3, #0]
    Mass_Memory_Size[0] = (Mass_Block_Count[0] * Mass_Block_Size[0]);
 8000596:	4b14      	ldr	r3, [pc, #80]	; (80005e8 <MAL_GetStatus+0xc0>)
 8000598:	681b      	ldr	r3, [r3, #0]
 800059a:	4a14      	ldr	r2, [pc, #80]	; (80005ec <MAL_GetStatus+0xc4>)
 800059c:	6812      	ldr	r2, [r2, #0]
 800059e:	fb02 f203 	mul.w	r2, r2, r3
 80005a2:	4b13      	ldr	r3, [pc, #76]	; (80005f0 <MAL_GetStatus+0xc8>)
 80005a4:	601a      	str	r2, [r3, #0]
#endif /* USE_STM3210E_EVAL */
      Mass_Memory_Size[0] = Mass_Block_Count[0] * Mass_Block_Size[0];
 80005a6:	4b10      	ldr	r3, [pc, #64]	; (80005e8 <MAL_GetStatus+0xc0>)
 80005a8:	681b      	ldr	r3, [r3, #0]
 80005aa:	4a10      	ldr	r2, [pc, #64]	; (80005ec <MAL_GetStatus+0xc4>)
 80005ac:	6812      	ldr	r2, [r2, #0]
 80005ae:	fb02 f203 	mul.w	r2, r2, r3
 80005b2:	4b0f      	ldr	r3, [pc, #60]	; (80005f0 <MAL_GetStatus+0xc8>)
 80005b4:	601a      	str	r2, [r3, #0]
	#ifdef CRT
      GREEN_LED_ON;
 80005b6:	480f      	ldr	r0, [pc, #60]	; (80005f4 <MAL_GetStatus+0xcc>)
 80005b8:	f44f 5180 	mov.w	r1, #4096	; 0x1000
 80005bc:	f04f 0201 	mov.w	r2, #1
 80005c0:	f004 f805 	bl	80045ce <GPIO_WriteBit>
	#else
      STM_EVAL_LEDOn(LED2);
	#endif
      return MAL_OK;
 80005c4:	f04f 0300 	mov.w	r3, #0
 80005c8:	e008      	b.n	80005dc <MAL_GetStatus+0xb4>
      return MAL_OK;
    }
  }
#endif /* USE_STM3210E_EVAL */
	#ifdef CRT
      GREEN_LED_OFF;
 80005ca:	480a      	ldr	r0, [pc, #40]	; (80005f4 <MAL_GetStatus+0xcc>)
 80005cc:	f44f 5180 	mov.w	r1, #4096	; 0x1000
 80005d0:	f04f 0200 	mov.w	r2, #0
 80005d4:	f003 fffb 	bl	80045ce <GPIO_WriteBit>
	#else
      STM_EVAL_LEDOff(LED2);
	#endif
  return MAL_FAIL;
 80005d8:	f04f 0301 	mov.w	r3, #1
}
 80005dc:	4618      	mov	r0, r3
 80005de:	f107 0740 	add.w	r7, r7, #64	; 0x40
 80005e2:	46bd      	mov	sp, r7
 80005e4:	bd80      	pop	{r7, pc}
 80005e6:	bf00      	nop
 80005e8:	20000248 	.word	0x20000248
 80005ec:	20000240 	.word	0x20000240
 80005f0:	20000238 	.word	0x20000238
 80005f4:	40010c00 	.word	0x40010c00

080005f8 <Read_Memory>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Read_Memory(uint8_t lun, uint32_t Memory_Offset, uint32_t Transfer_Length)
{
 80005f8:	b580      	push	{r7, lr}
 80005fa:	b084      	sub	sp, #16
 80005fc:	af00      	add	r7, sp, #0
 80005fe:	4603      	mov	r3, r0
 8000600:	60b9      	str	r1, [r7, #8]
 8000602:	607a      	str	r2, [r7, #4]
 8000604:	73fb      	strb	r3, [r7, #15]
  static uint32_t Offset, Length;

  if (TransferState == TXFR_IDLE )
 8000606:	4b4c      	ldr	r3, [pc, #304]	; (8000738 <Read_Memory+0x140>)
 8000608:	781b      	ldrb	r3, [r3, #0]
 800060a:	2b00      	cmp	r3, #0
 800060c:	d115      	bne.n	800063a <Read_Memory+0x42>
  {
    Offset = Memory_Offset * Mass_Block_Size[lun];
 800060e:	7bfa      	ldrb	r2, [r7, #15]
 8000610:	4b4a      	ldr	r3, [pc, #296]	; (800073c <Read_Memory+0x144>)
 8000612:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8000616:	68ba      	ldr	r2, [r7, #8]
 8000618:	fb02 f203 	mul.w	r2, r2, r3
 800061c:	4b48      	ldr	r3, [pc, #288]	; (8000740 <Read_Memory+0x148>)
 800061e:	601a      	str	r2, [r3, #0]
    Length = Transfer_Length * Mass_Block_Size[lun];
 8000620:	7bfa      	ldrb	r2, [r7, #15]
 8000622:	4b46      	ldr	r3, [pc, #280]	; (800073c <Read_Memory+0x144>)
 8000624:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8000628:	687a      	ldr	r2, [r7, #4]
 800062a:	fb02 f203 	mul.w	r2, r2, r3
 800062e:	4b45      	ldr	r3, [pc, #276]	; (8000744 <Read_Memory+0x14c>)
 8000630:	601a      	str	r2, [r3, #0]
    TransferState = TXFR_ONGOING;
 8000632:	4b41      	ldr	r3, [pc, #260]	; (8000738 <Read_Memory+0x140>)
 8000634:	f04f 0201 	mov.w	r2, #1
 8000638:	701a      	strb	r2, [r3, #0]
  }

  if (TransferState == TXFR_ONGOING )
 800063a:	4b3f      	ldr	r3, [pc, #252]	; (8000738 <Read_Memory+0x140>)
 800063c:	781b      	ldrb	r3, [r3, #0]
 800063e:	2b01      	cmp	r3, #1
 8000640:	d15c      	bne.n	80006fc <Read_Memory+0x104>
  {
    if (!Block_Read_count)
 8000642:	4b41      	ldr	r3, [pc, #260]	; (8000748 <Read_Memory+0x150>)
 8000644:	681b      	ldr	r3, [r3, #0]
 8000646:	2b00      	cmp	r3, #0
 8000648:	d121      	bne.n	800068e <Read_Memory+0x96>
    {
      MAL_Read(lun ,
 800064a:	4b3d      	ldr	r3, [pc, #244]	; (8000740 <Read_Memory+0x148>)
 800064c:	681a      	ldr	r2, [r3, #0]
               Offset ,
               Data_Buffer,
               Mass_Block_Size[lun]);
 800064e:	7bf9      	ldrb	r1, [r7, #15]
 8000650:	4b3a      	ldr	r3, [pc, #232]	; (800073c <Read_Memory+0x144>)
 8000652:	f853 3021 	ldr.w	r3, [r3, r1, lsl #2]

  if (TransferState == TXFR_ONGOING )
  {
    if (!Block_Read_count)
    {
      MAL_Read(lun ,
 8000656:	b29b      	uxth	r3, r3
 8000658:	7bf9      	ldrb	r1, [r7, #15]
 800065a:	4608      	mov	r0, r1
 800065c:	4611      	mov	r1, r2
 800065e:	4a3b      	ldr	r2, [pc, #236]	; (800074c <Read_Memory+0x154>)
 8000660:	f7ff ff40 	bl	80004e4 <MAL_Read>
               Offset ,
               Data_Buffer,
               Mass_Block_Size[lun]);

      USB_SIL_Write(EP1_IN, (uint8_t *)Data_Buffer, BULK_MAX_PACKET_SIZE);
 8000664:	4b39      	ldr	r3, [pc, #228]	; (800074c <Read_Memory+0x154>)
 8000666:	f04f 0081 	mov.w	r0, #129	; 0x81
 800066a:	4619      	mov	r1, r3
 800066c:	f04f 0240 	mov.w	r2, #64	; 0x40
 8000670:	f006 f878 	bl	8006764 <USB_SIL_Write>

      Block_Read_count = Mass_Block_Size[lun] - BULK_MAX_PACKET_SIZE;
 8000674:	7bfa      	ldrb	r2, [r7, #15]
 8000676:	4b31      	ldr	r3, [pc, #196]	; (800073c <Read_Memory+0x144>)
 8000678:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 800067c:	f1a3 0240 	sub.w	r2, r3, #64	; 0x40
 8000680:	4b31      	ldr	r3, [pc, #196]	; (8000748 <Read_Memory+0x150>)
 8000682:	601a      	str	r2, [r3, #0]
      Block_offset = BULK_MAX_PACKET_SIZE;
 8000684:	4b32      	ldr	r3, [pc, #200]	; (8000750 <Read_Memory+0x158>)
 8000686:	f04f 0240 	mov.w	r2, #64	; 0x40
 800068a:	601a      	str	r2, [r3, #0]
 800068c:	e016      	b.n	80006bc <Read_Memory+0xc4>
    }
    else
    {
      USB_SIL_Write(EP1_IN, (uint8_t *)Data_Buffer + Block_offset, BULK_MAX_PACKET_SIZE);
 800068e:	4a2f      	ldr	r2, [pc, #188]	; (800074c <Read_Memory+0x154>)
 8000690:	4b2f      	ldr	r3, [pc, #188]	; (8000750 <Read_Memory+0x158>)
 8000692:	681b      	ldr	r3, [r3, #0]
 8000694:	18d3      	adds	r3, r2, r3
 8000696:	f04f 0081 	mov.w	r0, #129	; 0x81
 800069a:	4619      	mov	r1, r3
 800069c:	f04f 0240 	mov.w	r2, #64	; 0x40
 80006a0:	f006 f860 	bl	8006764 <USB_SIL_Write>

      Block_Read_count -= BULK_MAX_PACKET_SIZE;
 80006a4:	4b28      	ldr	r3, [pc, #160]	; (8000748 <Read_Memory+0x150>)
 80006a6:	681b      	ldr	r3, [r3, #0]
 80006a8:	f1a3 0240 	sub.w	r2, r3, #64	; 0x40
 80006ac:	4b26      	ldr	r3, [pc, #152]	; (8000748 <Read_Memory+0x150>)
 80006ae:	601a      	str	r2, [r3, #0]
      Block_offset += BULK_MAX_PACKET_SIZE;
 80006b0:	4b27      	ldr	r3, [pc, #156]	; (8000750 <Read_Memory+0x158>)
 80006b2:	681b      	ldr	r3, [r3, #0]
 80006b4:	f103 0240 	add.w	r2, r3, #64	; 0x40
 80006b8:	4b25      	ldr	r3, [pc, #148]	; (8000750 <Read_Memory+0x158>)
 80006ba:	601a      	str	r2, [r3, #0]
    }

    SetEPTxCount(ENDP1, BULK_MAX_PACKET_SIZE);
 80006bc:	f04f 0001 	mov.w	r0, #1
 80006c0:	f04f 0140 	mov.w	r1, #64	; 0x40
 80006c4:	f005 fe2e 	bl	8006324 <SetEPTxCount>
#ifndef USE_STM3210C_EVAL
    SetEPTxStatus(ENDP1, EP_TX_VALID);
 80006c8:	f04f 0001 	mov.w	r0, #1
 80006cc:	f04f 0130 	mov.w	r1, #48	; 0x30
 80006d0:	f005 fc5a 	bl	8005f88 <SetEPTxStatus>
#endif    
    Offset += BULK_MAX_PACKET_SIZE;
 80006d4:	4b1a      	ldr	r3, [pc, #104]	; (8000740 <Read_Memory+0x148>)
 80006d6:	681b      	ldr	r3, [r3, #0]
 80006d8:	f103 0240 	add.w	r2, r3, #64	; 0x40
 80006dc:	4b18      	ldr	r3, [pc, #96]	; (8000740 <Read_Memory+0x148>)
 80006de:	601a      	str	r2, [r3, #0]
    Length -= BULK_MAX_PACKET_SIZE;
 80006e0:	4b18      	ldr	r3, [pc, #96]	; (8000744 <Read_Memory+0x14c>)
 80006e2:	681b      	ldr	r3, [r3, #0]
 80006e4:	f1a3 0240 	sub.w	r2, r3, #64	; 0x40
 80006e8:	4b16      	ldr	r3, [pc, #88]	; (8000744 <Read_Memory+0x14c>)
 80006ea:	601a      	str	r2, [r3, #0]

    CSW.dDataResidue -= BULK_MAX_PACKET_SIZE;
 80006ec:	4b19      	ldr	r3, [pc, #100]	; (8000754 <Read_Memory+0x15c>)
 80006ee:	689b      	ldr	r3, [r3, #8]
 80006f0:	f1a3 0240 	sub.w	r2, r3, #64	; 0x40
 80006f4:	4b17      	ldr	r3, [pc, #92]	; (8000754 <Read_Memory+0x15c>)
 80006f6:	609a      	str	r2, [r3, #8]
    Led_RW_ON();
 80006f8:	f7ff fdfc 	bl	80002f4 <Led_RW_ON>
  }
  if (Length == 0)
 80006fc:	4b11      	ldr	r3, [pc, #68]	; (8000744 <Read_Memory+0x14c>)
 80006fe:	681b      	ldr	r3, [r3, #0]
 8000700:	2b00      	cmp	r3, #0
 8000702:	d115      	bne.n	8000730 <Read_Memory+0x138>
  {
    Block_Read_count = 0;
 8000704:	4b10      	ldr	r3, [pc, #64]	; (8000748 <Read_Memory+0x150>)
 8000706:	f04f 0200 	mov.w	r2, #0
 800070a:	601a      	str	r2, [r3, #0]
    Block_offset = 0;
 800070c:	4b10      	ldr	r3, [pc, #64]	; (8000750 <Read_Memory+0x158>)
 800070e:	f04f 0200 	mov.w	r2, #0
 8000712:	601a      	str	r2, [r3, #0]
    Offset = 0;
 8000714:	4b0a      	ldr	r3, [pc, #40]	; (8000740 <Read_Memory+0x148>)
 8000716:	f04f 0200 	mov.w	r2, #0
 800071a:	601a      	str	r2, [r3, #0]
    Bot_State = BOT_DATA_IN_LAST;
 800071c:	4b0e      	ldr	r3, [pc, #56]	; (8000758 <Read_Memory+0x160>)
 800071e:	f04f 0203 	mov.w	r2, #3
 8000722:	701a      	strb	r2, [r3, #0]
    TransferState = TXFR_IDLE;
 8000724:	4b04      	ldr	r3, [pc, #16]	; (8000738 <Read_Memory+0x140>)
 8000726:	f04f 0200 	mov.w	r2, #0
 800072a:	701a      	strb	r2, [r3, #0]
    Led_RW_OFF();
 800072c:	f7ff fdee 	bl	800030c <Led_RW_OFF>
  }
}
 8000730:	f107 0710 	add.w	r7, r7, #16
 8000734:	46bd      	mov	sp, r7
 8000736:	bd80      	pop	{r7, pc}
 8000738:	20000184 	.word	0x20000184
 800073c:	20000240 	.word	0x20000240
 8000740:	20000190 	.word	0x20000190
 8000744:	20000194 	.word	0x20000194
 8000748:	2000017c 	.word	0x2000017c
 800074c:	20000258 	.word	0x20000258
 8000750:	20000250 	.word	0x20000250
 8000754:	20000480 	.word	0x20000480
 8000758:	2000047a 	.word	0x2000047a

0800075c <Write_Memory>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Write_Memory (uint8_t lun, uint32_t Memory_Offset, uint32_t Transfer_Length)
{
 800075c:	b580      	push	{r7, lr}
 800075e:	b086      	sub	sp, #24
 8000760:	af00      	add	r7, sp, #0
 8000762:	4603      	mov	r3, r0
 8000764:	60b9      	str	r1, [r7, #8]
 8000766:	607a      	str	r2, [r7, #4]
 8000768:	73fb      	strb	r3, [r7, #15]

  static uint32_t W_Offset, W_Length;

  uint32_t temp =  Counter + 64;
 800076a:	4b4e      	ldr	r3, [pc, #312]	; (80008a4 <Write_Memory+0x148>)
 800076c:	681b      	ldr	r3, [r3, #0]
 800076e:	f103 0340 	add.w	r3, r3, #64	; 0x40
 8000772:	617b      	str	r3, [r7, #20]

  if (TransferState == TXFR_IDLE )
 8000774:	4b4c      	ldr	r3, [pc, #304]	; (80008a8 <Write_Memory+0x14c>)
 8000776:	781b      	ldrb	r3, [r3, #0]
 8000778:	2b00      	cmp	r3, #0
 800077a:	d115      	bne.n	80007a8 <Write_Memory+0x4c>
  {
    W_Offset = Memory_Offset * Mass_Block_Size[lun];
 800077c:	7bfa      	ldrb	r2, [r7, #15]
 800077e:	4b4b      	ldr	r3, [pc, #300]	; (80008ac <Write_Memory+0x150>)
 8000780:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8000784:	68ba      	ldr	r2, [r7, #8]
 8000786:	fb02 f203 	mul.w	r2, r2, r3
 800078a:	4b49      	ldr	r3, [pc, #292]	; (80008b0 <Write_Memory+0x154>)
 800078c:	601a      	str	r2, [r3, #0]
    W_Length = Transfer_Length * Mass_Block_Size[lun];
 800078e:	7bfa      	ldrb	r2, [r7, #15]
 8000790:	4b46      	ldr	r3, [pc, #280]	; (80008ac <Write_Memory+0x150>)
 8000792:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8000796:	687a      	ldr	r2, [r7, #4]
 8000798:	fb02 f203 	mul.w	r2, r2, r3
 800079c:	4b45      	ldr	r3, [pc, #276]	; (80008b4 <Write_Memory+0x158>)
 800079e:	601a      	str	r2, [r3, #0]
    TransferState = TXFR_ONGOING;
 80007a0:	4b41      	ldr	r3, [pc, #260]	; (80008a8 <Write_Memory+0x14c>)
 80007a2:	f04f 0201 	mov.w	r2, #1
 80007a6:	701a      	strb	r2, [r3, #0]
  }

  if (TransferState == TXFR_ONGOING )
 80007a8:	4b3f      	ldr	r3, [pc, #252]	; (80008a8 <Write_Memory+0x14c>)
 80007aa:	781b      	ldrb	r3, [r3, #0]
 80007ac:	2b01      	cmp	r3, #1
 80007ae:	d15d      	bne.n	800086c <Write_Memory+0x110>
  {

    for (Idx = 0 ; Counter < temp; Counter++)
 80007b0:	4b41      	ldr	r3, [pc, #260]	; (80008b8 <Write_Memory+0x15c>)
 80007b2:	f04f 0200 	mov.w	r2, #0
 80007b6:	601a      	str	r2, [r3, #0]
 80007b8:	e012      	b.n	80007e0 <Write_Memory+0x84>
    {
      *((uint8_t *)Data_Buffer + Counter) = Bulk_Data_Buff[Idx++];
 80007ba:	4a40      	ldr	r2, [pc, #256]	; (80008bc <Write_Memory+0x160>)
 80007bc:	4b39      	ldr	r3, [pc, #228]	; (80008a4 <Write_Memory+0x148>)
 80007be:	681b      	ldr	r3, [r3, #0]
 80007c0:	18d2      	adds	r2, r2, r3
 80007c2:	4b3d      	ldr	r3, [pc, #244]	; (80008b8 <Write_Memory+0x15c>)
 80007c4:	681b      	ldr	r3, [r3, #0]
 80007c6:	493e      	ldr	r1, [pc, #248]	; (80008c0 <Write_Memory+0x164>)
 80007c8:	5cc9      	ldrb	r1, [r1, r3]
 80007ca:	7011      	strb	r1, [r2, #0]
 80007cc:	f103 0201 	add.w	r2, r3, #1
 80007d0:	4b39      	ldr	r3, [pc, #228]	; (80008b8 <Write_Memory+0x15c>)
 80007d2:	601a      	str	r2, [r3, #0]
  }

  if (TransferState == TXFR_ONGOING )
  {

    for (Idx = 0 ; Counter < temp; Counter++)
 80007d4:	4b33      	ldr	r3, [pc, #204]	; (80008a4 <Write_Memory+0x148>)
 80007d6:	681b      	ldr	r3, [r3, #0]
 80007d8:	f103 0201 	add.w	r2, r3, #1
 80007dc:	4b31      	ldr	r3, [pc, #196]	; (80008a4 <Write_Memory+0x148>)
 80007de:	601a      	str	r2, [r3, #0]
 80007e0:	4b30      	ldr	r3, [pc, #192]	; (80008a4 <Write_Memory+0x148>)
 80007e2:	681a      	ldr	r2, [r3, #0]
 80007e4:	697b      	ldr	r3, [r7, #20]
 80007e6:	429a      	cmp	r2, r3
 80007e8:	d3e7      	bcc.n	80007ba <Write_Memory+0x5e>
    {
      *((uint8_t *)Data_Buffer + Counter) = Bulk_Data_Buff[Idx++];
    }

    W_Offset += Data_Len;
 80007ea:	4b36      	ldr	r3, [pc, #216]	; (80008c4 <Write_Memory+0x168>)
 80007ec:	881b      	ldrh	r3, [r3, #0]
 80007ee:	461a      	mov	r2, r3
 80007f0:	4b2f      	ldr	r3, [pc, #188]	; (80008b0 <Write_Memory+0x154>)
 80007f2:	681b      	ldr	r3, [r3, #0]
 80007f4:	18d2      	adds	r2, r2, r3
 80007f6:	4b2e      	ldr	r3, [pc, #184]	; (80008b0 <Write_Memory+0x154>)
 80007f8:	601a      	str	r2, [r3, #0]
    W_Length -= Data_Len;
 80007fa:	4b2e      	ldr	r3, [pc, #184]	; (80008b4 <Write_Memory+0x158>)
 80007fc:	681a      	ldr	r2, [r3, #0]
 80007fe:	4b31      	ldr	r3, [pc, #196]	; (80008c4 <Write_Memory+0x168>)
 8000800:	881b      	ldrh	r3, [r3, #0]
 8000802:	1ad2      	subs	r2, r2, r3
 8000804:	4b2b      	ldr	r3, [pc, #172]	; (80008b4 <Write_Memory+0x158>)
 8000806:	601a      	str	r2, [r3, #0]

    if (!(W_Length % Mass_Block_Size[lun]))
 8000808:	4b2a      	ldr	r3, [pc, #168]	; (80008b4 <Write_Memory+0x158>)
 800080a:	681b      	ldr	r3, [r3, #0]
 800080c:	7bf9      	ldrb	r1, [r7, #15]
 800080e:	4a27      	ldr	r2, [pc, #156]	; (80008ac <Write_Memory+0x150>)
 8000810:	f852 2021 	ldr.w	r2, [r2, r1, lsl #2]
 8000814:	fbb3 f1f2 	udiv	r1, r3, r2
 8000818:	fb02 f201 	mul.w	r2, r2, r1
 800081c:	1a9b      	subs	r3, r3, r2
 800081e:	2b00      	cmp	r3, #0
 8000820:	d115      	bne.n	800084e <Write_Memory+0xf2>
    {
      Counter = 0;
 8000822:	4b20      	ldr	r3, [pc, #128]	; (80008a4 <Write_Memory+0x148>)
 8000824:	f04f 0200 	mov.w	r2, #0
 8000828:	601a      	str	r2, [r3, #0]
      MAL_Write(lun ,
 800082a:	4b21      	ldr	r3, [pc, #132]	; (80008b0 <Write_Memory+0x154>)
 800082c:	681a      	ldr	r2, [r3, #0]
                W_Offset - Mass_Block_Size[lun],
 800082e:	7bf9      	ldrb	r1, [r7, #15]
 8000830:	4b1e      	ldr	r3, [pc, #120]	; (80008ac <Write_Memory+0x150>)
 8000832:	f853 3021 	ldr.w	r3, [r3, r1, lsl #2]
    W_Length -= Data_Len;

    if (!(W_Length % Mass_Block_Size[lun]))
    {
      Counter = 0;
      MAL_Write(lun ,
 8000836:	1ad2      	subs	r2, r2, r3
                W_Offset - Mass_Block_Size[lun],
                Data_Buffer,
                Mass_Block_Size[lun]);
 8000838:	7bf9      	ldrb	r1, [r7, #15]
 800083a:	4b1c      	ldr	r3, [pc, #112]	; (80008ac <Write_Memory+0x150>)
 800083c:	f853 3021 	ldr.w	r3, [r3, r1, lsl #2]
    W_Length -= Data_Len;

    if (!(W_Length % Mass_Block_Size[lun]))
    {
      Counter = 0;
      MAL_Write(lun ,
 8000840:	b29b      	uxth	r3, r3
 8000842:	7bf9      	ldrb	r1, [r7, #15]
 8000844:	4608      	mov	r0, r1
 8000846:	4611      	mov	r1, r2
 8000848:	4a1c      	ldr	r2, [pc, #112]	; (80008bc <Write_Memory+0x160>)
 800084a:	f7ff fe29 	bl	80004a0 <MAL_Write>
                W_Offset - Mass_Block_Size[lun],
                Data_Buffer,
                Mass_Block_Size[lun]);
    }

    CSW.dDataResidue -= Data_Len;
 800084e:	4b1e      	ldr	r3, [pc, #120]	; (80008c8 <Write_Memory+0x16c>)
 8000850:	689a      	ldr	r2, [r3, #8]
 8000852:	4b1c      	ldr	r3, [pc, #112]	; (80008c4 <Write_Memory+0x168>)
 8000854:	881b      	ldrh	r3, [r3, #0]
 8000856:	1ad2      	subs	r2, r2, r3
 8000858:	4b1b      	ldr	r3, [pc, #108]	; (80008c8 <Write_Memory+0x16c>)
 800085a:	609a      	str	r2, [r3, #8]
  #ifndef STM32F10X_CL
    SetEPRxStatus(ENDP2, EP_RX_VALID); /* enable the next transaction*/   
 800085c:	f04f 0002 	mov.w	r0, #2
 8000860:	f44f 5140 	mov.w	r1, #12288	; 0x3000
 8000864:	f005 fbb0 	bl	8005fc8 <SetEPRxStatus>
  #endif /* STM32F10X_CL */

    Led_RW_ON();
 8000868:	f7ff fd44 	bl	80002f4 <Led_RW_ON>
  }

  if ((W_Length == 0) || (Bot_State == BOT_CSW_Send))
 800086c:	4b11      	ldr	r3, [pc, #68]	; (80008b4 <Write_Memory+0x158>)
 800086e:	681b      	ldr	r3, [r3, #0]
 8000870:	2b00      	cmp	r3, #0
 8000872:	d003      	beq.n	800087c <Write_Memory+0x120>
 8000874:	4b15      	ldr	r3, [pc, #84]	; (80008cc <Write_Memory+0x170>)
 8000876:	781b      	ldrb	r3, [r3, #0]
 8000878:	2b04      	cmp	r3, #4
 800087a:	d10f      	bne.n	800089c <Write_Memory+0x140>
  {
    Counter = 0;
 800087c:	4b09      	ldr	r3, [pc, #36]	; (80008a4 <Write_Memory+0x148>)
 800087e:	f04f 0200 	mov.w	r2, #0
 8000882:	601a      	str	r2, [r3, #0]
    Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
 8000884:	f04f 0000 	mov.w	r0, #0
 8000888:	f04f 0101 	mov.w	r1, #1
 800088c:	f000 fbd0 	bl	8001030 <Set_CSW>
    TransferState = TXFR_IDLE;
 8000890:	4b05      	ldr	r3, [pc, #20]	; (80008a8 <Write_Memory+0x14c>)
 8000892:	f04f 0200 	mov.w	r2, #0
 8000896:	701a      	strb	r2, [r3, #0]
    Led_RW_OFF();
 8000898:	f7ff fd38 	bl	800030c <Led_RW_OFF>
  }
}
 800089c:	f107 0718 	add.w	r7, r7, #24
 80008a0:	46bd      	mov	sp, r7
 80008a2:	bd80      	pop	{r7, pc}
 80008a4:	20000180 	.word	0x20000180
 80008a8:	20000184 	.word	0x20000184
 80008ac:	20000240 	.word	0x20000240
 80008b0:	20000188 	.word	0x20000188
 80008b4:	2000018c 	.word	0x2000018c
 80008b8:	20000254 	.word	0x20000254
 80008bc:	20000258 	.word	0x20000258
 80008c0:	20000494 	.word	0x20000494
 80008c4:	20000478 	.word	0x20000478
 80008c8:	20000480 	.word	0x20000480
 80008cc:	2000047a 	.word	0x2000047a

080008d0 <Mass_Storage_In>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Mass_Storage_In (void)
{
 80008d0:	b580      	push	{r7, lr}
 80008d2:	af00      	add	r7, sp, #0
  switch (Bot_State)
 80008d4:	4b1f      	ldr	r3, [pc, #124]	; (8000954 <Mass_Storage_In+0x84>)
 80008d6:	781b      	ldrb	r3, [r3, #0]
 80008d8:	f1a3 0302 	sub.w	r3, r3, #2
 80008dc:	2b03      	cmp	r3, #3
 80008de:	d834      	bhi.n	800094a <Mass_Storage_In+0x7a>
 80008e0:	a201      	add	r2, pc, #4	; (adr r2, 80008e8 <Mass_Storage_In+0x18>)
 80008e2:	f852 f023 	ldr.w	pc, [r2, r3, lsl #2]
 80008e6:	bf00      	nop
 80008e8:	0800090f 	.word	0x0800090f
 80008ec:	08000931 	.word	0x08000931
 80008f0:	080008f9 	.word	0x080008f9
 80008f4:	080008f9 	.word	0x080008f9
  {
    case BOT_CSW_Send:
    case BOT_ERROR:
      Bot_State = BOT_IDLE;
 80008f8:	4b16      	ldr	r3, [pc, #88]	; (8000954 <Mass_Storage_In+0x84>)
 80008fa:	f04f 0200 	mov.w	r2, #0
 80008fe:	701a      	strb	r2, [r3, #0]
    #ifndef STM32F10X_CL
      SetEPRxStatus(ENDP2, EP_RX_VALID);/* enable the Endpoint to receive the next cmd*/
 8000900:	f04f 0002 	mov.w	r0, #2
 8000904:	f44f 5140 	mov.w	r1, #12288	; 0x3000
 8000908:	f005 fb5e 	bl	8005fc8 <SetEPRxStatus>
      if (GetEPRxStatus(EP2_OUT) == EP_RX_STALL)
      {
        SetEPRxStatus(EP2_OUT, EP_RX_VALID);/* enable the Endpoint to receive the next cmd*/
      }
    #endif /* STM32F10X_CL */
      break;
 800090c:	e020      	b.n	8000950 <Mass_Storage_In+0x80>
    case BOT_DATA_IN:
      switch (CBW.CB[0])
 800090e:	4b12      	ldr	r3, [pc, #72]	; (8000958 <Mass_Storage_In+0x88>)
 8000910:	7bdb      	ldrb	r3, [r3, #15]
 8000912:	2b28      	cmp	r3, #40	; 0x28
 8000914:	d11b      	bne.n	800094e <Mass_Storage_In+0x7e>
      {
        case SCSI_READ10:
          SCSI_Read10_Cmd(CBW.bLUN , SCSI_LBA , SCSI_BlkLen);
 8000916:	4b10      	ldr	r3, [pc, #64]	; (8000958 <Mass_Storage_In+0x88>)
 8000918:	7b59      	ldrb	r1, [r3, #13]
 800091a:	4b10      	ldr	r3, [pc, #64]	; (800095c <Mass_Storage_In+0x8c>)
 800091c:	681a      	ldr	r2, [r3, #0]
 800091e:	4b10      	ldr	r3, [pc, #64]	; (8000960 <Mass_Storage_In+0x90>)
 8000920:	681b      	ldr	r3, [r3, #0]
 8000922:	4608      	mov	r0, r1
 8000924:	4611      	mov	r1, r2
 8000926:	461a      	mov	r2, r3
 8000928:	f001 f8ac 	bl	8001a84 <SCSI_Read10_Cmd>
          break;
 800092c:	bf00      	nop
      }
      break;
 800092e:	e00f      	b.n	8000950 <Mass_Storage_In+0x80>
    case BOT_DATA_IN_LAST:
      Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
 8000930:	f04f 0000 	mov.w	r0, #0
 8000934:	f04f 0101 	mov.w	r1, #1
 8000938:	f000 fb7a 	bl	8001030 <Set_CSW>
    #ifndef STM32F10X_CL
      SetEPRxStatus(ENDP2, EP_RX_VALID);
 800093c:	f04f 0002 	mov.w	r0, #2
 8000940:	f44f 5140 	mov.w	r1, #12288	; 0x3000
 8000944:	f005 fb40 	bl	8005fc8 <SetEPRxStatus>
      if (GetEPRxStatus(EP2_OUT) == EP_RX_STALL)
      {
        SetEPRxStatus(EP2_OUT, EP_RX_VALID);/* enable the Endpoint to receive the next cmd*/
      }      
    #endif /* STM32F10X_CL */
      break;
 8000948:	e002      	b.n	8000950 <Mass_Storage_In+0x80>

    default:
      break;
 800094a:	bf00      	nop
 800094c:	e000      	b.n	8000950 <Mass_Storage_In+0x80>
      {
        case SCSI_READ10:
          SCSI_Read10_Cmd(CBW.bLUN , SCSI_LBA , SCSI_BlkLen);
          break;
      }
      break;
 800094e:	bf00      	nop
      break;

    default:
      break;
  }
}
 8000950:	bd80      	pop	{r7, pc}
 8000952:	bf00      	nop
 8000954:	2000047a 	.word	0x2000047a
 8000958:	20000458 	.word	0x20000458
 800095c:	2000047c 	.word	0x2000047c
 8000960:	20000490 	.word	0x20000490

08000964 <Mass_Storage_Out>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Mass_Storage_Out (void)
{
 8000964:	b580      	push	{r7, lr}
 8000966:	b082      	sub	sp, #8
 8000968:	af00      	add	r7, sp, #0
  uint8_t CMD;
  CMD = CBW.CB[0];
 800096a:	4b28      	ldr	r3, [pc, #160]	; (8000a0c <Mass_Storage_Out+0xa8>)
 800096c:	7bdb      	ldrb	r3, [r3, #15]
 800096e:	71fb      	strb	r3, [r7, #7]

  Data_Len = USB_SIL_Read(EP2_OUT, Bulk_Data_Buff);
 8000970:	f04f 0002 	mov.w	r0, #2
 8000974:	4926      	ldr	r1, [pc, #152]	; (8000a10 <Mass_Storage_Out+0xac>)
 8000976:	f005 ff09 	bl	800678c <USB_SIL_Read>
 800097a:	4603      	mov	r3, r0
 800097c:	b29a      	uxth	r2, r3
 800097e:	4b25      	ldr	r3, [pc, #148]	; (8000a14 <Mass_Storage_Out+0xb0>)
 8000980:	801a      	strh	r2, [r3, #0]

  switch (Bot_State)
 8000982:	4b25      	ldr	r3, [pc, #148]	; (8000a18 <Mass_Storage_Out+0xb4>)
 8000984:	781b      	ldrb	r3, [r3, #0]
 8000986:	2b00      	cmp	r3, #0
 8000988:	d002      	beq.n	8000990 <Mass_Storage_Out+0x2c>
 800098a:	2b01      	cmp	r3, #1
 800098c:	d003      	beq.n	8000996 <Mass_Storage_Out+0x32>
 800098e:	e025      	b.n	80009dc <Mass_Storage_Out+0x78>
  {
    case BOT_IDLE:
      CBW_Decode();
 8000990:	f000 f848 	bl	8000a24 <CBW_Decode>
      break;
 8000994:	e036      	b.n	8000a04 <Mass_Storage_Out+0xa0>
    case BOT_DATA_OUT:
      if (CMD == SCSI_WRITE10)
 8000996:	79fb      	ldrb	r3, [r7, #7]
 8000998:	2b2a      	cmp	r3, #42	; 0x2a
 800099a:	d10b      	bne.n	80009b4 <Mass_Storage_Out+0x50>
      {
        SCSI_Write10_Cmd(CBW.bLUN , SCSI_LBA , SCSI_BlkLen);
 800099c:	4b1b      	ldr	r3, [pc, #108]	; (8000a0c <Mass_Storage_Out+0xa8>)
 800099e:	7b59      	ldrb	r1, [r3, #13]
 80009a0:	4b1e      	ldr	r3, [pc, #120]	; (8000a1c <Mass_Storage_Out+0xb8>)
 80009a2:	681a      	ldr	r2, [r3, #0]
 80009a4:	4b1e      	ldr	r3, [pc, #120]	; (8000a20 <Mass_Storage_Out+0xbc>)
 80009a6:	681b      	ldr	r3, [r3, #0]
 80009a8:	4608      	mov	r0, r1
 80009aa:	4611      	mov	r1, r2
 80009ac:	461a      	mov	r2, r3
 80009ae:	f001 f8b9 	bl	8001b24 <SCSI_Write10_Cmd>
        break;
 80009b2:	e027      	b.n	8000a04 <Mass_Storage_Out+0xa0>
      }
      Bot_Abort(DIR_OUT);
 80009b4:	f04f 0001 	mov.w	r0, #1
 80009b8:	f000 fb6a 	bl	8001090 <Bot_Abort>
      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
 80009bc:	4b13      	ldr	r3, [pc, #76]	; (8000a0c <Mass_Storage_Out+0xa8>)
 80009be:	7b5b      	ldrb	r3, [r3, #13]
 80009c0:	4618      	mov	r0, r3
 80009c2:	f04f 0105 	mov.w	r1, #5
 80009c6:	f04f 0224 	mov.w	r2, #36	; 0x24
 80009ca:	f001 f835 	bl	8001a38 <Set_Scsi_Sense_Data>
      Set_CSW (CSW_PHASE_ERROR, SEND_CSW_DISABLE);
 80009ce:	f04f 0002 	mov.w	r0, #2
 80009d2:	f04f 0100 	mov.w	r1, #0
 80009d6:	f000 fb2b 	bl	8001030 <Set_CSW>
      break;
 80009da:	e013      	b.n	8000a04 <Mass_Storage_Out+0xa0>
    default:
      Bot_Abort(BOTH_DIR);
 80009dc:	f04f 0002 	mov.w	r0, #2
 80009e0:	f000 fb56 	bl	8001090 <Bot_Abort>
      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
 80009e4:	4b09      	ldr	r3, [pc, #36]	; (8000a0c <Mass_Storage_Out+0xa8>)
 80009e6:	7b5b      	ldrb	r3, [r3, #13]
 80009e8:	4618      	mov	r0, r3
 80009ea:	f04f 0105 	mov.w	r1, #5
 80009ee:	f04f 0224 	mov.w	r2, #36	; 0x24
 80009f2:	f001 f821 	bl	8001a38 <Set_Scsi_Sense_Data>
      Set_CSW (CSW_PHASE_ERROR, SEND_CSW_DISABLE);
 80009f6:	f04f 0002 	mov.w	r0, #2
 80009fa:	f04f 0100 	mov.w	r1, #0
 80009fe:	f000 fb17 	bl	8001030 <Set_CSW>
      break;
 8000a02:	bf00      	nop
  }
}
 8000a04:	f107 0708 	add.w	r7, r7, #8
 8000a08:	46bd      	mov	sp, r7
 8000a0a:	bd80      	pop	{r7, pc}
 8000a0c:	20000458 	.word	0x20000458
 8000a10:	20000494 	.word	0x20000494
 8000a14:	20000478 	.word	0x20000478
 8000a18:	2000047a 	.word	0x2000047a
 8000a1c:	2000047c 	.word	0x2000047c
 8000a20:	20000490 	.word	0x20000490

08000a24 <CBW_Decode>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CBW_Decode(void)
{
 8000a24:	b580      	push	{r7, lr}
 8000a26:	b082      	sub	sp, #8
 8000a28:	af00      	add	r7, sp, #0
  uint32_t Counter;

  for (Counter = 0; Counter < Data_Len; Counter++)
 8000a2a:	f04f 0300 	mov.w	r3, #0
 8000a2e:	607b      	str	r3, [r7, #4]
 8000a30:	e00b      	b.n	8000a4a <CBW_Decode+0x26>
  {
    *((uint8_t *)&CBW + Counter) = Bulk_Data_Buff[Counter];
 8000a32:	4a43      	ldr	r2, [pc, #268]	; (8000b40 <CBW_Decode+0x11c>)
 8000a34:	687b      	ldr	r3, [r7, #4]
 8000a36:	18d3      	adds	r3, r2, r3
 8000a38:	4942      	ldr	r1, [pc, #264]	; (8000b44 <CBW_Decode+0x120>)
 8000a3a:	687a      	ldr	r2, [r7, #4]
 8000a3c:	188a      	adds	r2, r1, r2
 8000a3e:	7812      	ldrb	r2, [r2, #0]
 8000a40:	701a      	strb	r2, [r3, #0]
*******************************************************************************/
void CBW_Decode(void)
{
  uint32_t Counter;

  for (Counter = 0; Counter < Data_Len; Counter++)
 8000a42:	687b      	ldr	r3, [r7, #4]
 8000a44:	f103 0301 	add.w	r3, r3, #1
 8000a48:	607b      	str	r3, [r7, #4]
 8000a4a:	4b3f      	ldr	r3, [pc, #252]	; (8000b48 <CBW_Decode+0x124>)
 8000a4c:	881b      	ldrh	r3, [r3, #0]
 8000a4e:	461a      	mov	r2, r3
 8000a50:	687b      	ldr	r3, [r7, #4]
 8000a52:	429a      	cmp	r2, r3
 8000a54:	d8ed      	bhi.n	8000a32 <CBW_Decode+0xe>
  {
    *((uint8_t *)&CBW + Counter) = Bulk_Data_Buff[Counter];
  }
  CSW.dTag = CBW.dTag;
 8000a56:	4b3a      	ldr	r3, [pc, #232]	; (8000b40 <CBW_Decode+0x11c>)
 8000a58:	685a      	ldr	r2, [r3, #4]
 8000a5a:	4b3c      	ldr	r3, [pc, #240]	; (8000b4c <CBW_Decode+0x128>)
 8000a5c:	605a      	str	r2, [r3, #4]
  CSW.dDataResidue = CBW.dDataLength;
 8000a5e:	4b38      	ldr	r3, [pc, #224]	; (8000b40 <CBW_Decode+0x11c>)
 8000a60:	689a      	ldr	r2, [r3, #8]
 8000a62:	4b3a      	ldr	r3, [pc, #232]	; (8000b4c <CBW_Decode+0x128>)
 8000a64:	609a      	str	r2, [r3, #8]
  if (Data_Len != BOT_CBW_PACKET_LENGTH)
 8000a66:	4b38      	ldr	r3, [pc, #224]	; (8000b48 <CBW_Decode+0x124>)
 8000a68:	881b      	ldrh	r3, [r3, #0]
 8000a6a:	2b1f      	cmp	r3, #31
 8000a6c:	d017      	beq.n	8000a9e <CBW_Decode+0x7a>
  {
    Bot_Abort(BOTH_DIR);
 8000a6e:	f04f 0002 	mov.w	r0, #2
 8000a72:	f000 fb0d 	bl	8001090 <Bot_Abort>
    /* reset the CBW.dSignature to disable the clear feature until receiving a Mass storage reset*/
    CBW.dSignature = 0;
 8000a76:	4b32      	ldr	r3, [pc, #200]	; (8000b40 <CBW_Decode+0x11c>)
 8000a78:	f04f 0200 	mov.w	r2, #0
 8000a7c:	601a      	str	r2, [r3, #0]
    Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, PARAMETER_LIST_LENGTH_ERROR);
 8000a7e:	4b30      	ldr	r3, [pc, #192]	; (8000b40 <CBW_Decode+0x11c>)
 8000a80:	7b5b      	ldrb	r3, [r3, #13]
 8000a82:	4618      	mov	r0, r3
 8000a84:	f04f 0105 	mov.w	r1, #5
 8000a88:	f04f 021a 	mov.w	r2, #26
 8000a8c:	f000 ffd4 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8000a90:	f04f 0001 	mov.w	r0, #1
 8000a94:	f04f 0100 	mov.w	r1, #0
 8000a98:	f000 faca 	bl	8001030 <Set_CSW>
    return;
 8000a9c:	e298      	b.n	8000fd0 <CBW_Decode+0x5ac>
  }

  if ((CBW.CB[0] == SCSI_READ10 ) || (CBW.CB[0] == SCSI_WRITE10 ))
 8000a9e:	4b28      	ldr	r3, [pc, #160]	; (8000b40 <CBW_Decode+0x11c>)
 8000aa0:	7bdb      	ldrb	r3, [r3, #15]
 8000aa2:	2b28      	cmp	r3, #40	; 0x28
 8000aa4:	d003      	beq.n	8000aae <CBW_Decode+0x8a>
 8000aa6:	4b26      	ldr	r3, [pc, #152]	; (8000b40 <CBW_Decode+0x11c>)
 8000aa8:	7bdb      	ldrb	r3, [r3, #15]
 8000aaa:	2b2a      	cmp	r3, #42	; 0x2a
 8000aac:	d11f      	bne.n	8000aee <CBW_Decode+0xca>
  {
    /* Calculate Logical Block Address */
    SCSI_LBA = (CBW.CB[2] << 24) | (CBW.CB[3] << 16) | (CBW.CB[4] <<  8) | CBW.CB[5];
 8000aae:	4b24      	ldr	r3, [pc, #144]	; (8000b40 <CBW_Decode+0x11c>)
 8000ab0:	7c5b      	ldrb	r3, [r3, #17]
 8000ab2:	ea4f 6203 	mov.w	r2, r3, lsl #24
 8000ab6:	4b22      	ldr	r3, [pc, #136]	; (8000b40 <CBW_Decode+0x11c>)
 8000ab8:	7c9b      	ldrb	r3, [r3, #18]
 8000aba:	ea4f 4303 	mov.w	r3, r3, lsl #16
 8000abe:	431a      	orrs	r2, r3
 8000ac0:	4b1f      	ldr	r3, [pc, #124]	; (8000b40 <CBW_Decode+0x11c>)
 8000ac2:	7cdb      	ldrb	r3, [r3, #19]
 8000ac4:	ea4f 2303 	mov.w	r3, r3, lsl #8
 8000ac8:	431a      	orrs	r2, r3
 8000aca:	4b1d      	ldr	r3, [pc, #116]	; (8000b40 <CBW_Decode+0x11c>)
 8000acc:	7d1b      	ldrb	r3, [r3, #20]
 8000ace:	ea42 0303 	orr.w	r3, r2, r3
 8000ad2:	461a      	mov	r2, r3
 8000ad4:	4b1e      	ldr	r3, [pc, #120]	; (8000b50 <CBW_Decode+0x12c>)
 8000ad6:	601a      	str	r2, [r3, #0]
    /* Calculate the Number of Blocks to transfer */
    SCSI_BlkLen = (CBW.CB[7] <<  8) | CBW.CB[8];
 8000ad8:	4b19      	ldr	r3, [pc, #100]	; (8000b40 <CBW_Decode+0x11c>)
 8000ada:	7d9b      	ldrb	r3, [r3, #22]
 8000adc:	ea4f 2203 	mov.w	r2, r3, lsl #8
 8000ae0:	4b17      	ldr	r3, [pc, #92]	; (8000b40 <CBW_Decode+0x11c>)
 8000ae2:	7ddb      	ldrb	r3, [r3, #23]
 8000ae4:	ea42 0303 	orr.w	r3, r2, r3
 8000ae8:	461a      	mov	r2, r3
 8000aea:	4b1a      	ldr	r3, [pc, #104]	; (8000b54 <CBW_Decode+0x130>)
 8000aec:	601a      	str	r2, [r3, #0]
  }

  if (CBW.dSignature == BOT_CBW_SIGNATURE)
 8000aee:	4b14      	ldr	r3, [pc, #80]	; (8000b40 <CBW_Decode+0x11c>)
 8000af0:	681a      	ldr	r2, [r3, #0]
 8000af2:	4b19      	ldr	r3, [pc, #100]	; (8000b58 <CBW_Decode+0x134>)
 8000af4:	429a      	cmp	r2, r3
 8000af6:	f040 8258 	bne.w	8000faa <CBW_Decode+0x586>
  {
    /* Valid CBW */
    if ((CBW.bLUN > Max_Lun) || (CBW.bCBLength < 1) || (CBW.bCBLength > 16))
 8000afa:	4b11      	ldr	r3, [pc, #68]	; (8000b40 <CBW_Decode+0x11c>)
 8000afc:	7b5b      	ldrb	r3, [r3, #13]
 8000afe:	461a      	mov	r2, r3
 8000b00:	4b16      	ldr	r3, [pc, #88]	; (8000b5c <CBW_Decode+0x138>)
 8000b02:	681b      	ldr	r3, [r3, #0]
 8000b04:	429a      	cmp	r2, r3
 8000b06:	d807      	bhi.n	8000b18 <CBW_Decode+0xf4>
 8000b08:	4b0d      	ldr	r3, [pc, #52]	; (8000b40 <CBW_Decode+0x11c>)
 8000b0a:	7b9b      	ldrb	r3, [r3, #14]
 8000b0c:	2b00      	cmp	r3, #0
 8000b0e:	d003      	beq.n	8000b18 <CBW_Decode+0xf4>
 8000b10:	4b0b      	ldr	r3, [pc, #44]	; (8000b40 <CBW_Decode+0x11c>)
 8000b12:	7b9b      	ldrb	r3, [r3, #14]
 8000b14:	2b10      	cmp	r3, #16
 8000b16:	d923      	bls.n	8000b60 <CBW_Decode+0x13c>
    {
      Bot_Abort(BOTH_DIR);
 8000b18:	f04f 0002 	mov.w	r0, #2
 8000b1c:	f000 fab8 	bl	8001090 <Bot_Abort>
      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
 8000b20:	4b07      	ldr	r3, [pc, #28]	; (8000b40 <CBW_Decode+0x11c>)
 8000b22:	7b5b      	ldrb	r3, [r3, #13]
 8000b24:	4618      	mov	r0, r3
 8000b26:	f04f 0105 	mov.w	r1, #5
 8000b2a:	f04f 0224 	mov.w	r2, #36	; 0x24
 8000b2e:	f000 ff83 	bl	8001a38 <Set_Scsi_Sense_Data>
      Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8000b32:	f04f 0001 	mov.w	r0, #1
 8000b36:	f04f 0100 	mov.w	r1, #0
 8000b3a:	f000 fa79 	bl	8001030 <Set_CSW>
 8000b3e:	e247      	b.n	8000fd0 <CBW_Decode+0x5ac>
 8000b40:	20000458 	.word	0x20000458
 8000b44:	20000494 	.word	0x20000494
 8000b48:	20000478 	.word	0x20000478
 8000b4c:	20000480 	.word	0x20000480
 8000b50:	2000047c 	.word	0x2000047c
 8000b54:	20000490 	.word	0x20000490
 8000b58:	43425355 	.word	0x43425355
 8000b5c:	200001a8 	.word	0x200001a8
    }
    else
    {
      switch (CBW.CB[0])
 8000b60:	4bda      	ldr	r3, [pc, #872]	; (8000ecc <CBW_Decode+0x4a8>)
 8000b62:	7bdb      	ldrb	r3, [r3, #15]
 8000b64:	2baf      	cmp	r3, #175	; 0xaf
 8000b66:	f200 820c 	bhi.w	8000f82 <CBW_Decode+0x55e>
 8000b6a:	a201      	add	r2, pc, #4	; (adr r2, 8000b70 <CBW_Decode+0x14c>)
 8000b6c:	f852 f023 	ldr.w	pc, [r2, r3, lsl #2]
 8000b70:	08000e91 	.word	0x08000e91
 8000b74:	08000f83 	.word	0x08000f83
 8000b78:	08000f83 	.word	0x08000f83
 8000b7c:	08000e31 	.word	0x08000e31
 8000b80:	08000ee5 	.word	0x08000ee5
 8000b84:	08000f83 	.word	0x08000f83
 8000b88:	08000f83 	.word	0x08000f83
 8000b8c:	08000f83 	.word	0x08000f83
 8000b90:	08000f15 	.word	0x08000f15
 8000b94:	08000f83 	.word	0x08000f83
 8000b98:	08000f45 	.word	0x08000f45
 8000b9c:	08000f83 	.word	0x08000f83
 8000ba0:	08000f83 	.word	0x08000f83
 8000ba4:	08000f83 	.word	0x08000f83
 8000ba8:	08000f83 	.word	0x08000f83
 8000bac:	08000f83 	.word	0x08000f83
 8000bb0:	08000f83 	.word	0x08000f83
 8000bb4:	08000f83 	.word	0x08000f83
 8000bb8:	08000e3d 	.word	0x08000e3d
 8000bbc:	08000f83 	.word	0x08000f83
 8000bc0:	08000f83 	.word	0x08000f83
 8000bc4:	08000efd 	.word	0x08000efd
 8000bc8:	08000f83 	.word	0x08000f83
 8000bcc:	08000f83 	.word	0x08000f83
 8000bd0:	08000f83 	.word	0x08000f83
 8000bd4:	08000f83 	.word	0x08000f83
 8000bd8:	08000e61 	.word	0x08000e61
 8000bdc:	08000e49 	.word	0x08000e49
 8000be0:	08000f83 	.word	0x08000f83
 8000be4:	08000f09 	.word	0x08000f09
 8000be8:	08000e55 	.word	0x08000e55
 8000bec:	08000f83 	.word	0x08000f83
 8000bf0:	08000f83 	.word	0x08000f83
 8000bf4:	08000f83 	.word	0x08000f83
 8000bf8:	08000f83 	.word	0x08000f83
 8000bfc:	08000e79 	.word	0x08000e79
 8000c00:	08000f83 	.word	0x08000f83
 8000c04:	08000e85 	.word	0x08000e85
 8000c08:	08000f83 	.word	0x08000f83
 8000c0c:	08000f83 	.word	0x08000f83
 8000c10:	08000e9d 	.word	0x08000e9d
 8000c14:	08000f83 	.word	0x08000f83
 8000c18:	08000eb5 	.word	0x08000eb5
 8000c1c:	08000f83 	.word	0x08000f83
 8000c20:	08000f83 	.word	0x08000f83
 8000c24:	08000f83 	.word	0x08000f83
 8000c28:	08000f83 	.word	0x08000f83
 8000c2c:	08000ed9 	.word	0x08000ed9
 8000c30:	08000f83 	.word	0x08000f83
 8000c34:	08000f83 	.word	0x08000f83
 8000c38:	08000f83 	.word	0x08000f83
 8000c3c:	08000f83 	.word	0x08000f83
 8000c40:	08000f83 	.word	0x08000f83
 8000c44:	08000f83 	.word	0x08000f83
 8000c48:	08000f83 	.word	0x08000f83
 8000c4c:	08000f83 	.word	0x08000f83
 8000c50:	08000f83 	.word	0x08000f83
 8000c54:	08000f83 	.word	0x08000f83
 8000c58:	08000f83 	.word	0x08000f83
 8000c5c:	08000f83 	.word	0x08000f83
 8000c60:	08000f83 	.word	0x08000f83
 8000c64:	08000f83 	.word	0x08000f83
 8000c68:	08000f83 	.word	0x08000f83
 8000c6c:	08000f83 	.word	0x08000f83
 8000c70:	08000f83 	.word	0x08000f83
 8000c74:	08000f83 	.word	0x08000f83
 8000c78:	08000f83 	.word	0x08000f83
 8000c7c:	08000f83 	.word	0x08000f83
 8000c80:	08000f83 	.word	0x08000f83
 8000c84:	08000f83 	.word	0x08000f83
 8000c88:	08000f83 	.word	0x08000f83
 8000c8c:	08000f83 	.word	0x08000f83
 8000c90:	08000f83 	.word	0x08000f83
 8000c94:	08000f83 	.word	0x08000f83
 8000c98:	08000f83 	.word	0x08000f83
 8000c9c:	08000f83 	.word	0x08000f83
 8000ca0:	08000f83 	.word	0x08000f83
 8000ca4:	08000f83 	.word	0x08000f83
 8000ca8:	08000f83 	.word	0x08000f83
 8000cac:	08000f83 	.word	0x08000f83
 8000cb0:	08000f83 	.word	0x08000f83
 8000cb4:	08000f83 	.word	0x08000f83
 8000cb8:	08000f83 	.word	0x08000f83
 8000cbc:	08000f83 	.word	0x08000f83
 8000cc0:	08000f83 	.word	0x08000f83
 8000cc4:	08000ef1 	.word	0x08000ef1
 8000cc8:	08000f83 	.word	0x08000f83
 8000ccc:	08000f83 	.word	0x08000f83
 8000cd0:	08000f83 	.word	0x08000f83
 8000cd4:	08000f83 	.word	0x08000f83
 8000cd8:	08000e6d 	.word	0x08000e6d
 8000cdc:	08000f83 	.word	0x08000f83
 8000ce0:	08000f83 	.word	0x08000f83
 8000ce4:	08000f83 	.word	0x08000f83
 8000ce8:	08000f83 	.word	0x08000f83
 8000cec:	08000f83 	.word	0x08000f83
 8000cf0:	08000f83 	.word	0x08000f83
 8000cf4:	08000f83 	.word	0x08000f83
 8000cf8:	08000f83 	.word	0x08000f83
 8000cfc:	08000f83 	.word	0x08000f83
 8000d00:	08000f83 	.word	0x08000f83
 8000d04:	08000f83 	.word	0x08000f83
 8000d08:	08000f83 	.word	0x08000f83
 8000d0c:	08000f83 	.word	0x08000f83
 8000d10:	08000f83 	.word	0x08000f83
 8000d14:	08000f83 	.word	0x08000f83
 8000d18:	08000f83 	.word	0x08000f83
 8000d1c:	08000f83 	.word	0x08000f83
 8000d20:	08000f83 	.word	0x08000f83
 8000d24:	08000f83 	.word	0x08000f83
 8000d28:	08000f83 	.word	0x08000f83
 8000d2c:	08000f83 	.word	0x08000f83
 8000d30:	08000f83 	.word	0x08000f83
 8000d34:	08000f83 	.word	0x08000f83
 8000d38:	08000f83 	.word	0x08000f83
 8000d3c:	08000f83 	.word	0x08000f83
 8000d40:	08000f83 	.word	0x08000f83
 8000d44:	08000f83 	.word	0x08000f83
 8000d48:	08000f83 	.word	0x08000f83
 8000d4c:	08000f83 	.word	0x08000f83
 8000d50:	08000f83 	.word	0x08000f83
 8000d54:	08000f83 	.word	0x08000f83
 8000d58:	08000f83 	.word	0x08000f83
 8000d5c:	08000f83 	.word	0x08000f83
 8000d60:	08000f83 	.word	0x08000f83
 8000d64:	08000f83 	.word	0x08000f83
 8000d68:	08000f83 	.word	0x08000f83
 8000d6c:	08000f83 	.word	0x08000f83
 8000d70:	08000f83 	.word	0x08000f83
 8000d74:	08000f83 	.word	0x08000f83
 8000d78:	08000f83 	.word	0x08000f83
 8000d7c:	08000f83 	.word	0x08000f83
 8000d80:	08000f83 	.word	0x08000f83
 8000d84:	08000f83 	.word	0x08000f83
 8000d88:	08000f83 	.word	0x08000f83
 8000d8c:	08000f83 	.word	0x08000f83
 8000d90:	08000f2d 	.word	0x08000f2d
 8000d94:	08000f83 	.word	0x08000f83
 8000d98:	08000f5d 	.word	0x08000f5d
 8000d9c:	08000f83 	.word	0x08000f83
 8000da0:	08000f83 	.word	0x08000f83
 8000da4:	08000f83 	.word	0x08000f83
 8000da8:	08000f83 	.word	0x08000f83
 8000dac:	08000f75 	.word	0x08000f75
 8000db0:	08000f83 	.word	0x08000f83
 8000db4:	08000f83 	.word	0x08000f83
 8000db8:	08000f83 	.word	0x08000f83
 8000dbc:	08000f83 	.word	0x08000f83
 8000dc0:	08000f83 	.word	0x08000f83
 8000dc4:	08000f83 	.word	0x08000f83
 8000dc8:	08000f83 	.word	0x08000f83
 8000dcc:	08000f83 	.word	0x08000f83
 8000dd0:	08000f83 	.word	0x08000f83
 8000dd4:	08000f83 	.word	0x08000f83
 8000dd8:	08000f83 	.word	0x08000f83
 8000ddc:	08000f83 	.word	0x08000f83
 8000de0:	08000f83 	.word	0x08000f83
 8000de4:	08000f83 	.word	0x08000f83
 8000de8:	08000f39 	.word	0x08000f39
 8000dec:	08000f83 	.word	0x08000f83
 8000df0:	08000f83 	.word	0x08000f83
 8000df4:	08000f83 	.word	0x08000f83
 8000df8:	08000f83 	.word	0x08000f83
 8000dfc:	08000f83 	.word	0x08000f83
 8000e00:	08000f83 	.word	0x08000f83
 8000e04:	08000f83 	.word	0x08000f83
 8000e08:	08000f83 	.word	0x08000f83
 8000e0c:	08000f83 	.word	0x08000f83
 8000e10:	08000f21 	.word	0x08000f21
 8000e14:	08000f83 	.word	0x08000f83
 8000e18:	08000f51 	.word	0x08000f51
 8000e1c:	08000f83 	.word	0x08000f83
 8000e20:	08000f83 	.word	0x08000f83
 8000e24:	08000f83 	.word	0x08000f83
 8000e28:	08000f83 	.word	0x08000f83
 8000e2c:	08000f69 	.word	0x08000f69
      {
        case SCSI_REQUEST_SENSE:
          SCSI_RequestSense_Cmd (CBW.bLUN);
 8000e30:	4b26      	ldr	r3, [pc, #152]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e32:	7b5b      	ldrb	r3, [r3, #13]
 8000e34:	4618      	mov	r0, r3
 8000e36:	f000 fde1 	bl	80019fc <SCSI_RequestSense_Cmd>
          break;
 8000e3a:	e0c9      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_INQUIRY:
          SCSI_Inquiry_Cmd(CBW.bLUN);
 8000e3c:	4b23      	ldr	r3, [pc, #140]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e3e:	7b5b      	ldrb	r3, [r3, #13]
 8000e40:	4618      	mov	r0, r3
 8000e42:	f000 fc9b 	bl	800177c <SCSI_Inquiry_Cmd>
          break;
 8000e46:	e0c3      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_START_STOP_UNIT:
          SCSI_Start_Stop_Unit_Cmd(CBW.bLUN);
 8000e48:	4b20      	ldr	r3, [pc, #128]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e4a:	7b5b      	ldrb	r3, [r3, #13]
 8000e4c:	4618      	mov	r0, r3
 8000e4e:	f000 fe09 	bl	8001a64 <SCSI_Start_Stop_Unit_Cmd>
          break;
 8000e52:	e0bd      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_ALLOW_MEDIUM_REMOVAL:
          SCSI_Start_Stop_Unit_Cmd(CBW.bLUN);
 8000e54:	4b1d      	ldr	r3, [pc, #116]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e56:	7b5b      	ldrb	r3, [r3, #13]
 8000e58:	4618      	mov	r0, r3
 8000e5a:	f000 fe03 	bl	8001a64 <SCSI_Start_Stop_Unit_Cmd>
          break;
 8000e5e:	e0b7      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_MODE_SENSE6:
          SCSI_ModeSense6_Cmd (CBW.bLUN);
 8000e60:	4b1a      	ldr	r3, [pc, #104]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e62:	7b5b      	ldrb	r3, [r3, #13]
 8000e64:	4618      	mov	r0, r3
 8000e66:	f000 fda9 	bl	80019bc <SCSI_ModeSense6_Cmd>
          break;
 8000e6a:	e0b1      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_MODE_SENSE10:
          SCSI_ModeSense10_Cmd (CBW.bLUN);
 8000e6c:	4b17      	ldr	r3, [pc, #92]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e6e:	7b5b      	ldrb	r3, [r3, #13]
 8000e70:	4618      	mov	r0, r3
 8000e72:	f000 fdb3 	bl	80019dc <SCSI_ModeSense10_Cmd>
          break;
 8000e76:	e0ab      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_READ_FORMAT_CAPACITIES:
          SCSI_ReadFormatCapacity_Cmd(CBW.bLUN);
 8000e78:	4b14      	ldr	r3, [pc, #80]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e7a:	7b5b      	ldrb	r3, [r3, #13]
 8000e7c:	4618      	mov	r0, r3
 8000e7e:	f000 fcb3 	bl	80017e8 <SCSI_ReadFormatCapacity_Cmd>
          break;
 8000e82:	e0a5      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_READ_CAPACITY10:
          SCSI_ReadCapacity10_Cmd(CBW.bLUN);
 8000e84:	4b11      	ldr	r3, [pc, #68]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e86:	7b5b      	ldrb	r3, [r3, #13]
 8000e88:	4618      	mov	r0, r3
 8000e8a:	f000 fd19 	bl	80018c0 <SCSI_ReadCapacity10_Cmd>
          break;
 8000e8e:	e09f      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_TEST_UNIT_READY:
          SCSI_TestUnitReady_Cmd(CBW.bLUN);
 8000e90:	4b0e      	ldr	r3, [pc, #56]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e92:	7b5b      	ldrb	r3, [r3, #13]
 8000e94:	4618      	mov	r0, r3
 8000e96:	f000 feef 	bl	8001c78 <SCSI_TestUnitReady_Cmd>
          break;
 8000e9a:	e099      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_READ10:
          SCSI_Read10_Cmd(CBW.bLUN, SCSI_LBA , SCSI_BlkLen);
 8000e9c:	4b0b      	ldr	r3, [pc, #44]	; (8000ecc <CBW_Decode+0x4a8>)
 8000e9e:	7b59      	ldrb	r1, [r3, #13]
 8000ea0:	4b0b      	ldr	r3, [pc, #44]	; (8000ed0 <CBW_Decode+0x4ac>)
 8000ea2:	681a      	ldr	r2, [r3, #0]
 8000ea4:	4b0b      	ldr	r3, [pc, #44]	; (8000ed4 <CBW_Decode+0x4b0>)
 8000ea6:	681b      	ldr	r3, [r3, #0]
 8000ea8:	4608      	mov	r0, r1
 8000eaa:	4611      	mov	r1, r2
 8000eac:	461a      	mov	r2, r3
 8000eae:	f000 fde9 	bl	8001a84 <SCSI_Read10_Cmd>
          break;
 8000eb2:	e08d      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_WRITE10:
          SCSI_Write10_Cmd(CBW.bLUN, SCSI_LBA , SCSI_BlkLen);
 8000eb4:	4b05      	ldr	r3, [pc, #20]	; (8000ecc <CBW_Decode+0x4a8>)
 8000eb6:	7b59      	ldrb	r1, [r3, #13]
 8000eb8:	4b05      	ldr	r3, [pc, #20]	; (8000ed0 <CBW_Decode+0x4ac>)
 8000eba:	681a      	ldr	r2, [r3, #0]
 8000ebc:	4b05      	ldr	r3, [pc, #20]	; (8000ed4 <CBW_Decode+0x4b0>)
 8000ebe:	681b      	ldr	r3, [r3, #0]
 8000ec0:	4608      	mov	r0, r1
 8000ec2:	4611      	mov	r1, r2
 8000ec4:	461a      	mov	r2, r3
 8000ec6:	f000 fe2d 	bl	8001b24 <SCSI_Write10_Cmd>
          break;
 8000eca:	e081      	b.n	8000fd0 <CBW_Decode+0x5ac>
 8000ecc:	20000458 	.word	0x20000458
 8000ed0:	2000047c 	.word	0x2000047c
 8000ed4:	20000490 	.word	0x20000490
        case SCSI_VERIFY10:
          SCSI_Verify10_Cmd(CBW.bLUN);
 8000ed8:	4b3f      	ldr	r3, [pc, #252]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000eda:	7b5b      	ldrb	r3, [r3, #13]
 8000edc:	4618      	mov	r0, r3
 8000ede:	f000 fe71 	bl	8001bc4 <SCSI_Verify10_Cmd>
          break;
 8000ee2:	e075      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_FORMAT_UNIT:
          SCSI_Format_Cmd(CBW.bLUN);
 8000ee4:	4b3c      	ldr	r3, [pc, #240]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000ee6:	7b5b      	ldrb	r3, [r3, #13]
 8000ee8:	4618      	mov	r0, r3
 8000eea:	f000 fef1 	bl	8001cd0 <SCSI_Format_Cmd>
          break;
 8000eee:	e06f      	b.n	8000fd0 <CBW_Decode+0x5ac>
          /*Unsupported command*/

        case SCSI_MODE_SELECT10:
          SCSI_Mode_Select10_Cmd(CBW.bLUN);
 8000ef0:	4b39      	ldr	r3, [pc, #228]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000ef2:	7b5b      	ldrb	r3, [r3, #13]
 8000ef4:	4618      	mov	r0, r3
 8000ef6:	f000 ff11 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000efa:	e069      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_MODE_SELECT6:
          SCSI_Mode_Select6_Cmd(CBW.bLUN);
 8000efc:	4b36      	ldr	r3, [pc, #216]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000efe:	7b5b      	ldrb	r3, [r3, #13]
 8000f00:	4618      	mov	r0, r3
 8000f02:	f000 ff0b 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f06:	e063      	b.n	8000fd0 <CBW_Decode+0x5ac>

        case SCSI_SEND_DIAGNOSTIC:
          SCSI_Send_Diagnostic_Cmd(CBW.bLUN);
 8000f08:	4b33      	ldr	r3, [pc, #204]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f0a:	7b5b      	ldrb	r3, [r3, #13]
 8000f0c:	4618      	mov	r0, r3
 8000f0e:	f000 ff05 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f12:	e05d      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_READ6:
          SCSI_Read6_Cmd(CBW.bLUN);
 8000f14:	4b30      	ldr	r3, [pc, #192]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f16:	7b5b      	ldrb	r3, [r3, #13]
 8000f18:	4618      	mov	r0, r3
 8000f1a:	f000 feff 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f1e:	e057      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_READ12:
          SCSI_Read12_Cmd(CBW.bLUN);
 8000f20:	4b2d      	ldr	r3, [pc, #180]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f22:	7b5b      	ldrb	r3, [r3, #13]
 8000f24:	4618      	mov	r0, r3
 8000f26:	f000 fef9 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f2a:	e051      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_READ16:
          SCSI_Read16_Cmd(CBW.bLUN);
 8000f2c:	4b2a      	ldr	r3, [pc, #168]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f2e:	7b5b      	ldrb	r3, [r3, #13]
 8000f30:	4618      	mov	r0, r3
 8000f32:	f000 fef3 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f36:	e04b      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_READ_CAPACITY16:
          SCSI_READ_CAPACITY16_Cmd(CBW.bLUN);
 8000f38:	4b27      	ldr	r3, [pc, #156]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f3a:	7b5b      	ldrb	r3, [r3, #13]
 8000f3c:	4618      	mov	r0, r3
 8000f3e:	f000 feed 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f42:	e045      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_WRITE6:
          SCSI_Write6_Cmd(CBW.bLUN);
 8000f44:	4b24      	ldr	r3, [pc, #144]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f46:	7b5b      	ldrb	r3, [r3, #13]
 8000f48:	4618      	mov	r0, r3
 8000f4a:	f000 fee7 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f4e:	e03f      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_WRITE12:
          SCSI_Write12_Cmd(CBW.bLUN);
 8000f50:	4b21      	ldr	r3, [pc, #132]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f52:	7b5b      	ldrb	r3, [r3, #13]
 8000f54:	4618      	mov	r0, r3
 8000f56:	f000 fee1 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f5a:	e039      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_WRITE16:
          SCSI_Write16_Cmd(CBW.bLUN);
 8000f5c:	4b1e      	ldr	r3, [pc, #120]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f5e:	7b5b      	ldrb	r3, [r3, #13]
 8000f60:	4618      	mov	r0, r3
 8000f62:	f000 fedb 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f66:	e033      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_VERIFY12:
          SCSI_Verify12_Cmd(CBW.bLUN);
 8000f68:	4b1b      	ldr	r3, [pc, #108]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f6a:	7b5b      	ldrb	r3, [r3, #13]
 8000f6c:	4618      	mov	r0, r3
 8000f6e:	f000 fed5 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f72:	e02d      	b.n	8000fd0 <CBW_Decode+0x5ac>
        case SCSI_VERIFY16:
          SCSI_Verify16_Cmd(CBW.bLUN);
 8000f74:	4b18      	ldr	r3, [pc, #96]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f76:	7b5b      	ldrb	r3, [r3, #13]
 8000f78:	4618      	mov	r0, r3
 8000f7a:	f000 fecf 	bl	8001d1c <SCSI_Invalid_Cmd>
          break;
 8000f7e:	bf00      	nop
 8000f80:	e026      	b.n	8000fd0 <CBW_Decode+0x5ac>

        default:
        {
          Bot_Abort(BOTH_DIR);
 8000f82:	f04f 0002 	mov.w	r0, #2
 8000f86:	f000 f883 	bl	8001090 <Bot_Abort>
          Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_COMMAND);
 8000f8a:	4b13      	ldr	r3, [pc, #76]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000f8c:	7b5b      	ldrb	r3, [r3, #13]
 8000f8e:	4618      	mov	r0, r3
 8000f90:	f04f 0105 	mov.w	r1, #5
 8000f94:	f04f 0220 	mov.w	r2, #32
 8000f98:	f000 fd4e 	bl	8001a38 <Set_Scsi_Sense_Data>
          Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8000f9c:	f04f 0001 	mov.w	r0, #1
 8000fa0:	f04f 0100 	mov.w	r1, #0
 8000fa4:	f000 f844 	bl	8001030 <Set_CSW>
 8000fa8:	e012      	b.n	8000fd0 <CBW_Decode+0x5ac>
    }
  }
  else
  {
    /* Invalid CBW */
    Bot_Abort(BOTH_DIR);
 8000faa:	f04f 0002 	mov.w	r0, #2
 8000fae:	f000 f86f 	bl	8001090 <Bot_Abort>
    Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_COMMAND);
 8000fb2:	4b09      	ldr	r3, [pc, #36]	; (8000fd8 <CBW_Decode+0x5b4>)
 8000fb4:	7b5b      	ldrb	r3, [r3, #13]
 8000fb6:	4618      	mov	r0, r3
 8000fb8:	f04f 0105 	mov.w	r1, #5
 8000fbc:	f04f 0220 	mov.w	r2, #32
 8000fc0:	f000 fd3a 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8000fc4:	f04f 0001 	mov.w	r0, #1
 8000fc8:	f04f 0100 	mov.w	r1, #0
 8000fcc:	f000 f830 	bl	8001030 <Set_CSW>
  }
}
 8000fd0:	f107 0708 	add.w	r7, r7, #8
 8000fd4:	46bd      	mov	sp, r7
 8000fd6:	bd80      	pop	{r7, pc}
 8000fd8:	20000458 	.word	0x20000458

08000fdc <Transfer_Data_Request>:
*                  uint16_t Data_Length : the number of Bytes to transfer.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Transfer_Data_Request(uint8_t* Data_Pointer, uint16_t Data_Len)
{
 8000fdc:	b580      	push	{r7, lr}
 8000fde:	b082      	sub	sp, #8
 8000fe0:	af00      	add	r7, sp, #0
 8000fe2:	6078      	str	r0, [r7, #4]
 8000fe4:	460b      	mov	r3, r1
 8000fe6:	807b      	strh	r3, [r7, #2]
  USB_SIL_Write(EP1_IN, Data_Pointer, Data_Len);
 8000fe8:	887b      	ldrh	r3, [r7, #2]
 8000fea:	f04f 0081 	mov.w	r0, #129	; 0x81
 8000fee:	6879      	ldr	r1, [r7, #4]
 8000ff0:	461a      	mov	r2, r3
 8000ff2:	f005 fbb7 	bl	8006764 <USB_SIL_Write>

#ifndef USE_STM3210C_EVAL
    SetEPTxStatus(ENDP1, EP_TX_VALID);
 8000ff6:	f04f 0001 	mov.w	r0, #1
 8000ffa:	f04f 0130 	mov.w	r1, #48	; 0x30
 8000ffe:	f004 ffc3 	bl	8005f88 <SetEPTxStatus>
#endif  
  Bot_State = BOT_DATA_IN_LAST;
 8001002:	4b09      	ldr	r3, [pc, #36]	; (8001028 <Transfer_Data_Request+0x4c>)
 8001004:	f04f 0203 	mov.w	r2, #3
 8001008:	701a      	strb	r2, [r3, #0]
  CSW.dDataResidue -= Data_Len;
 800100a:	4b08      	ldr	r3, [pc, #32]	; (800102c <Transfer_Data_Request+0x50>)
 800100c:	689a      	ldr	r2, [r3, #8]
 800100e:	887b      	ldrh	r3, [r7, #2]
 8001010:	1ad2      	subs	r2, r2, r3
 8001012:	4b06      	ldr	r3, [pc, #24]	; (800102c <Transfer_Data_Request+0x50>)
 8001014:	609a      	str	r2, [r3, #8]
  CSW.bStatus = CSW_CMD_PASSED;
 8001016:	4b05      	ldr	r3, [pc, #20]	; (800102c <Transfer_Data_Request+0x50>)
 8001018:	f04f 0200 	mov.w	r2, #0
 800101c:	731a      	strb	r2, [r3, #12]
}
 800101e:	f107 0708 	add.w	r7, r7, #8
 8001022:	46bd      	mov	sp, r7
 8001024:	bd80      	pop	{r7, pc}
 8001026:	bf00      	nop
 8001028:	2000047a 	.word	0x2000047a
 800102c:	20000480 	.word	0x20000480

08001030 <Set_CSW>:
*                  or CSW_PHASE_ERROR.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_CSW (uint8_t CSW_Status, uint8_t Send_Permission)
{
 8001030:	b580      	push	{r7, lr}
 8001032:	b082      	sub	sp, #8
 8001034:	af00      	add	r7, sp, #0
 8001036:	4602      	mov	r2, r0
 8001038:	460b      	mov	r3, r1
 800103a:	71fa      	strb	r2, [r7, #7]
 800103c:	71bb      	strb	r3, [r7, #6]
  CSW.dSignature = BOT_CSW_SIGNATURE;
 800103e:	4b11      	ldr	r3, [pc, #68]	; (8001084 <Set_CSW+0x54>)
 8001040:	4a11      	ldr	r2, [pc, #68]	; (8001088 <Set_CSW+0x58>)
 8001042:	601a      	str	r2, [r3, #0]
  CSW.bStatus = CSW_Status;
 8001044:	4b0f      	ldr	r3, [pc, #60]	; (8001084 <Set_CSW+0x54>)
 8001046:	79fa      	ldrb	r2, [r7, #7]
 8001048:	731a      	strb	r2, [r3, #12]

  USB_SIL_Write(EP1_IN, ((uint8_t *)& CSW), CSW_DATA_LENGTH);
 800104a:	4b0e      	ldr	r3, [pc, #56]	; (8001084 <Set_CSW+0x54>)
 800104c:	f04f 0081 	mov.w	r0, #129	; 0x81
 8001050:	4619      	mov	r1, r3
 8001052:	f04f 020d 	mov.w	r2, #13
 8001056:	f005 fb85 	bl	8006764 <USB_SIL_Write>

  Bot_State = BOT_ERROR;
 800105a:	4b0c      	ldr	r3, [pc, #48]	; (800108c <Set_CSW+0x5c>)
 800105c:	f04f 0205 	mov.w	r2, #5
 8001060:	701a      	strb	r2, [r3, #0]
  if (Send_Permission)
 8001062:	79bb      	ldrb	r3, [r7, #6]
 8001064:	2b00      	cmp	r3, #0
 8001066:	d009      	beq.n	800107c <Set_CSW+0x4c>
  {
    Bot_State = BOT_CSW_Send;
 8001068:	4b08      	ldr	r3, [pc, #32]	; (800108c <Set_CSW+0x5c>)
 800106a:	f04f 0204 	mov.w	r2, #4
 800106e:	701a      	strb	r2, [r3, #0]
#ifndef USE_STM3210C_EVAL
    SetEPTxStatus(ENDP1, EP_TX_VALID);
 8001070:	f04f 0001 	mov.w	r0, #1
 8001074:	f04f 0130 	mov.w	r1, #48	; 0x30
 8001078:	f004 ff86 	bl	8005f88 <SetEPTxStatus>
#endif  
  }

}
 800107c:	f107 0708 	add.w	r7, r7, #8
 8001080:	46bd      	mov	sp, r7
 8001082:	bd80      	pop	{r7, pc}
 8001084:	20000480 	.word	0x20000480
 8001088:	53425355 	.word	0x53425355
 800108c:	2000047a 	.word	0x2000047a

08001090 <Bot_Abort>:
* Input          : Endpoint direction IN, OUT or both directions
* Output         : None.
* Return         : None.
*******************************************************************************/
void Bot_Abort(uint8_t Direction)
{
 8001090:	b580      	push	{r7, lr}
 8001092:	b082      	sub	sp, #8
 8001094:	af00      	add	r7, sp, #0
 8001096:	4603      	mov	r3, r0
 8001098:	71fb      	strb	r3, [r7, #7]
  switch (Direction)
 800109a:	79fb      	ldrb	r3, [r7, #7]
 800109c:	2b01      	cmp	r3, #1
 800109e:	d00a      	beq.n	80010b6 <Bot_Abort+0x26>
 80010a0:	2b02      	cmp	r3, #2
 80010a2:	d00f      	beq.n	80010c4 <Bot_Abort+0x34>
 80010a4:	2b00      	cmp	r3, #0
 80010a6:	d11b      	bne.n	80010e0 <Bot_Abort+0x50>
  {
    case DIR_IN :
      SetEPTxStatus(ENDP1, EP_TX_STALL);
 80010a8:	f04f 0001 	mov.w	r0, #1
 80010ac:	f04f 0110 	mov.w	r1, #16
 80010b0:	f004 ff6a 	bl	8005f88 <SetEPTxStatus>
      break;
 80010b4:	e015      	b.n	80010e2 <Bot_Abort+0x52>
    case DIR_OUT :
      SetEPRxStatus(ENDP2, EP_RX_STALL);
 80010b6:	f04f 0002 	mov.w	r0, #2
 80010ba:	f44f 5180 	mov.w	r1, #4096	; 0x1000
 80010be:	f004 ff83 	bl	8005fc8 <SetEPRxStatus>
      break;
 80010c2:	e00e      	b.n	80010e2 <Bot_Abort+0x52>
    case BOTH_DIR :
      SetEPTxStatus(ENDP1, EP_TX_STALL);
 80010c4:	f04f 0001 	mov.w	r0, #1
 80010c8:	f04f 0110 	mov.w	r1, #16
 80010cc:	f004 ff5c 	bl	8005f88 <SetEPTxStatus>
      SetEPRxStatus(ENDP2, EP_RX_STALL);
 80010d0:	f04f 0002 	mov.w	r0, #2
 80010d4:	f44f 5180 	mov.w	r1, #4096	; 0x1000
 80010d8:	f004 ff76 	bl	8005fc8 <SetEPRxStatus>
      break;
 80010dc:	bf00      	nop
 80010de:	e000      	b.n	80010e2 <Bot_Abort+0x52>
    default:
      break;
 80010e0:	bf00      	nop
  }
}
 80010e2:	f107 0708 	add.w	r7, r7, #8
 80010e6:	46bd      	mov	sp, r7
 80010e8:	bd80      	pop	{r7, pc}
 80010ea:	bf00      	nop

080010ec <EP1_IN_Callback>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
 80010ec:	b580      	push	{r7, lr}
 80010ee:	af00      	add	r7, sp, #0
  Mass_Storage_In();
 80010f0:	f7ff fbee 	bl	80008d0 <Mass_Storage_In>
}
 80010f4:	bd80      	pop	{r7, pc}
 80010f6:	bf00      	nop

080010f8 <EP2_OUT_Callback>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP2_OUT_Callback(void)
{
 80010f8:	b580      	push	{r7, lr}
 80010fa:	af00      	add	r7, sp, #0
  Mass_Storage_Out();
 80010fc:	f7ff fc32 	bl	8000964 <Mass_Storage_Out>
}
 8001100:	bd80      	pop	{r7, pc}
 8001102:	bf00      	nop

08001104 <USB_Istr>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Istr(void)
{
 8001104:	b580      	push	{r7, lr}
 8001106:	af00      	add	r7, sp, #0

  wIstr = _GetISTR();
 8001108:	4b12      	ldr	r3, [pc, #72]	; (8001154 <USB_Istr+0x50>)
 800110a:	681b      	ldr	r3, [r3, #0]
 800110c:	b29a      	uxth	r2, r3
 800110e:	4b12      	ldr	r3, [pc, #72]	; (8001158 <USB_Istr+0x54>)
 8001110:	801a      	strh	r2, [r3, #0]

#if (IMR_MSK & ISTR_CTR)
  if (wIstr & ISTR_CTR & wInterrupt_Mask)
 8001112:	4b11      	ldr	r3, [pc, #68]	; (8001158 <USB_Istr+0x54>)
 8001114:	881b      	ldrh	r3, [r3, #0]
 8001116:	b29b      	uxth	r3, r3
 8001118:	f403 4200 	and.w	r2, r3, #32768	; 0x8000
 800111c:	4b0f      	ldr	r3, [pc, #60]	; (800115c <USB_Istr+0x58>)
 800111e:	881b      	ldrh	r3, [r3, #0]
 8001120:	ea02 0303 	and.w	r3, r2, r3
 8001124:	2b00      	cmp	r3, #0
 8001126:	d001      	beq.n	800112c <USB_Istr+0x28>
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    CTR_LP();
 8001128:	f004 fd42 	bl	8005bb0 <CTR_LP>
#endif
  }
#endif  
  /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_RESET)
  if (wIstr & ISTR_RESET & wInterrupt_Mask)
 800112c:	4b0a      	ldr	r3, [pc, #40]	; (8001158 <USB_Istr+0x54>)
 800112e:	881b      	ldrh	r3, [r3, #0]
 8001130:	b29b      	uxth	r3, r3
 8001132:	f403 6280 	and.w	r2, r3, #1024	; 0x400
 8001136:	4b09      	ldr	r3, [pc, #36]	; (800115c <USB_Istr+0x58>)
 8001138:	881b      	ldrh	r3, [r3, #0]
 800113a:	ea02 0303 	and.w	r3, r2, r3
 800113e:	2b00      	cmp	r3, #0
 8001140:	d006      	beq.n	8001150 <USB_Istr+0x4c>
  {
    _SetISTR((uint16_t)CLR_RESET);
 8001142:	4b04      	ldr	r3, [pc, #16]	; (8001154 <USB_Istr+0x50>)
 8001144:	f64f 32ff 	movw	r2, #64511	; 0xfbff
 8001148:	601a      	str	r2, [r3, #0]
    Device_Property.Reset();
 800114a:	4b05      	ldr	r3, [pc, #20]	; (8001160 <USB_Istr+0x5c>)
 800114c:	685b      	ldr	r3, [r3, #4]
 800114e:	4798      	blx	r3
#ifdef ESOF_CALLBACK
    ESOF_Callback();
#endif
  }
#endif
} /* USB_Istr */
 8001150:	bd80      	pop	{r7, pc}
 8001152:	bf00      	nop
 8001154:	40005c44 	.word	0x40005c44
 8001158:	200004d4 	.word	0x200004d4
 800115c:	20000510 	.word	0x20000510
 8001160:	200000d0 	.word	0x200000d0

08001164 <MASS_init>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void MASS_init()
{
 8001164:	b580      	push	{r7, lr}
 8001166:	af00      	add	r7, sp, #0
  /* Update the serial number string descriptor with the data from the unique
  ID*/
  Get_SerialNum();
 8001168:	f7ff f8fe 	bl	8000368 <Get_SerialNum>

  pInformation->Current_Configuration = 0;
 800116c:	4b06      	ldr	r3, [pc, #24]	; (8001188 <MASS_init+0x24>)
 800116e:	681b      	ldr	r3, [r3, #0]
 8001170:	f04f 0200 	mov.w	r2, #0
 8001174:	729a      	strb	r2, [r3, #10]

  /* Connect the device */
  PowerOn();
 8001176:	f000 f9f5 	bl	8001564 <PowerOn>

  /* Perform basic device initialization operations */
  USB_SIL_Init();
 800117a:	f005 fae3 	bl	8006744 <USB_SIL_Init>

  bDeviceState = UNCONNECTED;
 800117e:	4b03      	ldr	r3, [pc, #12]	; (800118c <MASS_init+0x28>)
 8001180:	f04f 0200 	mov.w	r2, #0
 8001184:	601a      	str	r2, [r3, #0]
}
 8001186:	bd80      	pop	{r7, pc}
 8001188:	2000050c 	.word	0x2000050c
 800118c:	200001ac 	.word	0x200001ac

08001190 <MASS_Reset>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void MASS_Reset()
{
 8001190:	b580      	push	{r7, lr}
 8001192:	af00      	add	r7, sp, #0
  /* Set the device as not configured */
  Device_Info.Current_Configuration = 0;
 8001194:	4b45      	ldr	r3, [pc, #276]	; (80012ac <MASS_Reset+0x11c>)
 8001196:	f04f 0200 	mov.w	r2, #0
 800119a:	729a      	strb	r2, [r3, #10]

  /* Current Feature initialization */
  pInformation->Current_Feature = MASS_ConfigDescriptor[7];
 800119c:	4b44      	ldr	r3, [pc, #272]	; (80012b0 <MASS_Reset+0x120>)
 800119e:	681b      	ldr	r3, [r3, #0]
 80011a0:	4a44      	ldr	r2, [pc, #272]	; (80012b4 <MASS_Reset+0x124>)
 80011a2:	79d2      	ldrb	r2, [r2, #7]
 80011a4:	725a      	strb	r2, [r3, #9]
  /* Init EP2 OUT as Bulk endpoint */
  OTG_DEV_EP_Init(EP2_OUT, OTG_DEV_EP_TYPE_BULK, BULK_MAX_PACKET_SIZE); 
  
#else 

  SetBTABLE(BTABLE_ADDRESS);
 80011a6:	f04f 0000 	mov.w	r0, #0
 80011aa:	f004 feb5 	bl	8005f18 <SetBTABLE>

  /* Initialize Endpoint 0 */
  SetEPType(ENDP0, EP_CONTROL);
 80011ae:	f04f 0000 	mov.w	r0, #0
 80011b2:	f44f 7100 	mov.w	r1, #512	; 0x200
 80011b6:	f004 fecf 	bl	8005f58 <SetEPType>
  SetEPTxStatus(ENDP0, EP_TX_NAK);
 80011ba:	f04f 0000 	mov.w	r0, #0
 80011be:	f04f 0120 	mov.w	r1, #32
 80011c2:	f004 fee1 	bl	8005f88 <SetEPTxStatus>
  SetEPRxAddr(ENDP0, ENDP0_RXADDR);
 80011c6:	f04f 0000 	mov.w	r0, #0
 80011ca:	f04f 0118 	mov.w	r1, #24
 80011ce:	f005 f875 	bl	80062bc <SetEPRxAddr>
  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
 80011d2:	4b39      	ldr	r3, [pc, #228]	; (80012b8 <MASS_Reset+0x128>)
 80011d4:	f893 302c 	ldrb.w	r3, [r3, #44]	; 0x2c
 80011d8:	f04f 0000 	mov.w	r0, #0
 80011dc:	4619      	mov	r1, r3
 80011de:	f005 f8c7 	bl	8006370 <SetEPRxCount>
  SetEPTxAddr(ENDP0, ENDP0_TXADDR);
 80011e2:	f04f 0000 	mov.w	r0, #0
 80011e6:	f04f 0158 	mov.w	r1, #88	; 0x58
 80011ea:	f005 f855 	bl	8006298 <SetEPTxAddr>
  Clear_Status_Out(ENDP0);
 80011ee:	f04f 0000 	mov.w	r0, #0
 80011f2:	f004 ff75 	bl	80060e0 <Clear_Status_Out>
  SetEPRxValid(ENDP0);
 80011f6:	f04f 0000 	mov.w	r0, #0
 80011fa:	f004 ff3f 	bl	800607c <SetEPRxValid>

  /* Initialize Endpoint 1 */
  SetEPType(ENDP1, EP_BULK);
 80011fe:	f04f 0001 	mov.w	r0, #1
 8001202:	f04f 0100 	mov.w	r1, #0
 8001206:	f004 fea7 	bl	8005f58 <SetEPType>
  SetEPTxAddr(ENDP1, ENDP1_TXADDR);
 800120a:	f04f 0001 	mov.w	r0, #1
 800120e:	f04f 0198 	mov.w	r1, #152	; 0x98
 8001212:	f005 f841 	bl	8006298 <SetEPTxAddr>
  SetEPTxStatus(ENDP1, EP_TX_NAK);
 8001216:	f04f 0001 	mov.w	r0, #1
 800121a:	f04f 0120 	mov.w	r1, #32
 800121e:	f004 feb3 	bl	8005f88 <SetEPTxStatus>
  SetEPRxStatus(ENDP1, EP_RX_DIS);
 8001222:	f04f 0001 	mov.w	r0, #1
 8001226:	f04f 0100 	mov.w	r1, #0
 800122a:	f004 fecd 	bl	8005fc8 <SetEPRxStatus>

  /* Initialize Endpoint 2 */
  SetEPType(ENDP2, EP_BULK);
 800122e:	f04f 0002 	mov.w	r0, #2
 8001232:	f04f 0100 	mov.w	r1, #0
 8001236:	f004 fe8f 	bl	8005f58 <SetEPType>
  SetEPRxAddr(ENDP2, ENDP2_RXADDR);
 800123a:	f04f 0002 	mov.w	r0, #2
 800123e:	f04f 01d8 	mov.w	r1, #216	; 0xd8
 8001242:	f005 f83b 	bl	80062bc <SetEPRxAddr>
  SetEPRxCount(ENDP2, Device_Property.MaxPacketSize);
 8001246:	4b1c      	ldr	r3, [pc, #112]	; (80012b8 <MASS_Reset+0x128>)
 8001248:	f893 302c 	ldrb.w	r3, [r3, #44]	; 0x2c
 800124c:	f04f 0002 	mov.w	r0, #2
 8001250:	4619      	mov	r1, r3
 8001252:	f005 f88d 	bl	8006370 <SetEPRxCount>
  SetEPRxStatus(ENDP2, EP_RX_VALID);
 8001256:	f04f 0002 	mov.w	r0, #2
 800125a:	f44f 5140 	mov.w	r1, #12288	; 0x3000
 800125e:	f004 feb3 	bl	8005fc8 <SetEPRxStatus>
  SetEPTxStatus(ENDP2, EP_TX_DIS);
 8001262:	f04f 0002 	mov.w	r0, #2
 8001266:	f04f 0100 	mov.w	r1, #0
 800126a:	f004 fe8d 	bl	8005f88 <SetEPTxStatus>


  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
 800126e:	4b12      	ldr	r3, [pc, #72]	; (80012b8 <MASS_Reset+0x128>)
 8001270:	f893 302c 	ldrb.w	r3, [r3, #44]	; 0x2c
 8001274:	f04f 0000 	mov.w	r0, #0
 8001278:	4619      	mov	r1, r3
 800127a:	f005 f879 	bl	8006370 <SetEPRxCount>
  SetEPRxValid(ENDP0);
 800127e:	f04f 0000 	mov.w	r0, #0
 8001282:	f004 fefb 	bl	800607c <SetEPRxValid>

  /* Set the device to response on default address */
  SetDeviceAddress(0);
 8001286:	f04f 0000 	mov.w	r0, #0
 800128a:	f004 fbe5 	bl	8005a58 <SetDeviceAddress>
#endif /* STM32F10X_CL */

  bDeviceState = ATTACHED;
 800128e:	4b0b      	ldr	r3, [pc, #44]	; (80012bc <MASS_Reset+0x12c>)
 8001290:	f04f 0201 	mov.w	r2, #1
 8001294:	601a      	str	r2, [r3, #0]

  CBW.dSignature = BOT_CBW_SIGNATURE;
 8001296:	4b0a      	ldr	r3, [pc, #40]	; (80012c0 <MASS_Reset+0x130>)
 8001298:	4a0a      	ldr	r2, [pc, #40]	; (80012c4 <MASS_Reset+0x134>)
 800129a:	601a      	str	r2, [r3, #0]
  Bot_State = BOT_IDLE;
 800129c:	4b0a      	ldr	r3, [pc, #40]	; (80012c8 <MASS_Reset+0x138>)
 800129e:	f04f 0200 	mov.w	r2, #0
 80012a2:	701a      	strb	r2, [r3, #0]

  USB_NotConfigured_LED();
 80012a4:	f7ff f84a 	bl	800033c <USB_NotConfigured_LED>
}
 80012a8:	bd80      	pop	{r7, pc}
 80012aa:	bf00      	nop
 80012ac:	200004ec 	.word	0x200004ec
 80012b0:	2000050c 	.word	0x2000050c
 80012b4:	080076c4 	.word	0x080076c4
 80012b8:	200000d0 	.word	0x200000d0
 80012bc:	200001ac 	.word	0x200001ac
 80012c0:	20000458 	.word	0x20000458
 80012c4:	43425355 	.word	0x43425355
 80012c8:	2000047a 	.word	0x2000047a

080012cc <Mass_Storage_SetConfiguration>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Mass_Storage_SetConfiguration(void)
{
 80012cc:	b580      	push	{r7, lr}
 80012ce:	af00      	add	r7, sp, #0
  if (pInformation->Current_Configuration != 0)
 80012d0:	4b0a      	ldr	r3, [pc, #40]	; (80012fc <Mass_Storage_SetConfiguration+0x30>)
 80012d2:	681b      	ldr	r3, [r3, #0]
 80012d4:	7a9b      	ldrb	r3, [r3, #10]
 80012d6:	2b00      	cmp	r3, #0
 80012d8:	d00f      	beq.n	80012fa <Mass_Storage_SetConfiguration+0x2e>
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
 80012da:	4b09      	ldr	r3, [pc, #36]	; (8001300 <Mass_Storage_SetConfiguration+0x34>)
 80012dc:	f04f 0205 	mov.w	r2, #5
 80012e0:	601a      	str	r2, [r3, #0]
    OTG_DEV_EP_Init(EP1_IN, OTG_DEV_EP_TYPE_BULK, BULK_MAX_PACKET_SIZE);
  
    /* Init EP2 OUT as Bulk endpoint */
    OTG_DEV_EP_Init(EP2_OUT, OTG_DEV_EP_TYPE_BULK, BULK_MAX_PACKET_SIZE);     
#else    
    ClearDTOG_TX(ENDP1);
 80012e2:	f04f 0001 	mov.w	r0, #1
 80012e6:	f004 ffa7 	bl	8006238 <ClearDTOG_TX>
    ClearDTOG_RX(ENDP2);
 80012ea:	f04f 0002 	mov.w	r0, #2
 80012ee:	f004 ff8f 	bl	8006210 <ClearDTOG_RX>
#endif /* STM32F10X_CL */

    Bot_State = BOT_IDLE; /* set the Bot state machine to the IDLE state */
 80012f2:	4b04      	ldr	r3, [pc, #16]	; (8001304 <Mass_Storage_SetConfiguration+0x38>)
 80012f4:	f04f 0200 	mov.w	r2, #0
 80012f8:	701a      	strb	r2, [r3, #0]
  }
}
 80012fa:	bd80      	pop	{r7, pc}
 80012fc:	2000050c 	.word	0x2000050c
 8001300:	200001ac 	.word	0x200001ac
 8001304:	2000047a 	.word	0x2000047a

08001308 <Mass_Storage_ClearFeature>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Mass_Storage_ClearFeature(void)
{
 8001308:	b580      	push	{r7, lr}
 800130a:	af00      	add	r7, sp, #0
  /* when the host send a CBW with invalid signature or invalid length the two
     Endpoints (IN & OUT) shall stall until receiving a Mass Storage Reset     */
  if (CBW.dSignature != BOT_CBW_SIGNATURE)
 800130c:	4b04      	ldr	r3, [pc, #16]	; (8001320 <Mass_Storage_ClearFeature+0x18>)
 800130e:	681a      	ldr	r2, [r3, #0]
 8001310:	4b04      	ldr	r3, [pc, #16]	; (8001324 <Mass_Storage_ClearFeature+0x1c>)
 8001312:	429a      	cmp	r2, r3
 8001314:	d003      	beq.n	800131e <Mass_Storage_ClearFeature+0x16>
    Bot_Abort(BOTH_DIR);
 8001316:	f04f 0002 	mov.w	r0, #2
 800131a:	f7ff feb9 	bl	8001090 <Bot_Abort>
}
 800131e:	bd80      	pop	{r7, pc}
 8001320:	20000458 	.word	0x20000458
 8001324:	43425355 	.word	0x43425355

08001328 <Mass_Storage_SetDeviceAddress>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Mass_Storage_SetDeviceAddress (void)
{
 8001328:	b480      	push	{r7}
 800132a:	af00      	add	r7, sp, #0
  bDeviceState = ADDRESSED;
 800132c:	4b03      	ldr	r3, [pc, #12]	; (800133c <Mass_Storage_SetDeviceAddress+0x14>)
 800132e:	f04f 0204 	mov.w	r2, #4
 8001332:	601a      	str	r2, [r3, #0]
}
 8001334:	46bd      	mov	sp, r7
 8001336:	bc80      	pop	{r7}
 8001338:	4770      	bx	lr
 800133a:	bf00      	nop
 800133c:	200001ac 	.word	0x200001ac

08001340 <MASS_Status_In>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void MASS_Status_In(void)
{
 8001340:	b480      	push	{r7}
 8001342:	af00      	add	r7, sp, #0
  return;
}
 8001344:	46bd      	mov	sp, r7
 8001346:	bc80      	pop	{r7}
 8001348:	4770      	bx	lr
 800134a:	bf00      	nop

0800134c <MASS_Status_Out>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void MASS_Status_Out(void)
{
 800134c:	b480      	push	{r7}
 800134e:	af00      	add	r7, sp, #0
  return;
}
 8001350:	46bd      	mov	sp, r7
 8001352:	bc80      	pop	{r7}
 8001354:	4770      	bx	lr
 8001356:	bf00      	nop

08001358 <MASS_Data_Setup>:
* Input          : RequestNo.
* Output         : None.
* Return         : RESULT.
*******************************************************************************/
RESULT MASS_Data_Setup(uint8_t RequestNo)
{
 8001358:	b580      	push	{r7, lr}
 800135a:	b084      	sub	sp, #16
 800135c:	af00      	add	r7, sp, #0
 800135e:	4603      	mov	r3, r0
 8001360:	71fb      	strb	r3, [r7, #7]
  uint8_t    *(*CopyRoutine)(uint16_t);

  CopyRoutine = NULL;
 8001362:	f04f 0300 	mov.w	r3, #0
 8001366:	60fb      	str	r3, [r7, #12]
  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
 8001368:	4b1c      	ldr	r3, [pc, #112]	; (80013dc <MASS_Data_Setup+0x84>)
 800136a:	681b      	ldr	r3, [r3, #0]
 800136c:	781b      	ldrb	r3, [r3, #0]
 800136e:	f003 037f 	and.w	r3, r3, #127	; 0x7f
 8001372:	2b21      	cmp	r3, #33	; 0x21
 8001374:	d117      	bne.n	80013a6 <MASS_Data_Setup+0x4e>
      && (RequestNo == GET_MAX_LUN) && (pInformation->USBwValue == 0)
 8001376:	79fb      	ldrb	r3, [r7, #7]
 8001378:	2bfe      	cmp	r3, #254	; 0xfe
 800137a:	d114      	bne.n	80013a6 <MASS_Data_Setup+0x4e>
 800137c:	4b17      	ldr	r3, [pc, #92]	; (80013dc <MASS_Data_Setup+0x84>)
 800137e:	681b      	ldr	r3, [r3, #0]
 8001380:	885b      	ldrh	r3, [r3, #2]
 8001382:	2b00      	cmp	r3, #0
 8001384:	d10f      	bne.n	80013a6 <MASS_Data_Setup+0x4e>
      && (pInformation->USBwIndex == 0) && (pInformation->USBwLength == 0x01))
 8001386:	4b15      	ldr	r3, [pc, #84]	; (80013dc <MASS_Data_Setup+0x84>)
 8001388:	681b      	ldr	r3, [r3, #0]
 800138a:	889b      	ldrh	r3, [r3, #4]
 800138c:	2b00      	cmp	r3, #0
 800138e:	d10a      	bne.n	80013a6 <MASS_Data_Setup+0x4e>
 8001390:	4b12      	ldr	r3, [pc, #72]	; (80013dc <MASS_Data_Setup+0x84>)
 8001392:	681b      	ldr	r3, [r3, #0]
 8001394:	88db      	ldrh	r3, [r3, #6]
 8001396:	2b01      	cmp	r3, #1
 8001398:	d105      	bne.n	80013a6 <MASS_Data_Setup+0x4e>
  {
    CopyRoutine = Get_Max_Lun;
 800139a:	4b11      	ldr	r3, [pc, #68]	; (80013e0 <MASS_Data_Setup+0x88>)
 800139c:	60fb      	str	r3, [r7, #12]
  else
  {
    return USB_UNSUPPORT;
  }

  if (CopyRoutine == NULL)
 800139e:	68fb      	ldr	r3, [r7, #12]
 80013a0:	2b00      	cmp	r3, #0
 80013a2:	d003      	beq.n	80013ac <MASS_Data_Setup+0x54>
 80013a4:	e005      	b.n	80013b2 <MASS_Data_Setup+0x5a>
  {
    CopyRoutine = Get_Max_Lun;
  }
  else
  {
    return USB_UNSUPPORT;
 80013a6:	f04f 0302 	mov.w	r3, #2
 80013aa:	e011      	b.n	80013d0 <MASS_Data_Setup+0x78>
  }

  if (CopyRoutine == NULL)
  {
    return USB_UNSUPPORT;
 80013ac:	f04f 0302 	mov.w	r3, #2
 80013b0:	e00e      	b.n	80013d0 <MASS_Data_Setup+0x78>
  }

  pInformation->Ctrl_Info.CopyData = CopyRoutine;
 80013b2:	4b0a      	ldr	r3, [pc, #40]	; (80013dc <MASS_Data_Setup+0x84>)
 80013b4:	681b      	ldr	r3, [r3, #0]
 80013b6:	68fa      	ldr	r2, [r7, #12]
 80013b8:	619a      	str	r2, [r3, #24]
  pInformation->Ctrl_Info.Usb_wOffset = 0;
 80013ba:	4b08      	ldr	r3, [pc, #32]	; (80013dc <MASS_Data_Setup+0x84>)
 80013bc:	681b      	ldr	r3, [r3, #0]
 80013be:	f04f 0200 	mov.w	r2, #0
 80013c2:	825a      	strh	r2, [r3, #18]
  (*CopyRoutine)(0);
 80013c4:	68fb      	ldr	r3, [r7, #12]
 80013c6:	f04f 0000 	mov.w	r0, #0
 80013ca:	4798      	blx	r3

  return USB_SUCCESS;
 80013cc:	f04f 0300 	mov.w	r3, #0

}
 80013d0:	4618      	mov	r0, r3
 80013d2:	f107 0710 	add.w	r7, r7, #16
 80013d6:	46bd      	mov	sp, r7
 80013d8:	bd80      	pop	{r7, pc}
 80013da:	bf00      	nop
 80013dc:	2000050c 	.word	0x2000050c
 80013e0:	0800152d 	.word	0x0800152d

080013e4 <MASS_NoData_Setup>:
* Input          : RequestNo.
* Output         : None.
* Return         : RESULT.
*******************************************************************************/
RESULT MASS_NoData_Setup(uint8_t RequestNo)
{
 80013e4:	b580      	push	{r7, lr}
 80013e6:	b082      	sub	sp, #8
 80013e8:	af00      	add	r7, sp, #0
 80013ea:	4603      	mov	r3, r0
 80013ec:	71fb      	strb	r3, [r7, #7]
  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
 80013ee:	4b19      	ldr	r3, [pc, #100]	; (8001454 <MASS_NoData_Setup+0x70>)
 80013f0:	681b      	ldr	r3, [r3, #0]
 80013f2:	781b      	ldrb	r3, [r3, #0]
 80013f4:	f003 037f 	and.w	r3, r3, #127	; 0x7f
 80013f8:	2b21      	cmp	r3, #33	; 0x21
 80013fa:	d123      	bne.n	8001444 <MASS_NoData_Setup+0x60>
      && (RequestNo == MASS_STORAGE_RESET) && (pInformation->USBwValue == 0)
 80013fc:	79fb      	ldrb	r3, [r7, #7]
 80013fe:	2bff      	cmp	r3, #255	; 0xff
 8001400:	d120      	bne.n	8001444 <MASS_NoData_Setup+0x60>
 8001402:	4b14      	ldr	r3, [pc, #80]	; (8001454 <MASS_NoData_Setup+0x70>)
 8001404:	681b      	ldr	r3, [r3, #0]
 8001406:	885b      	ldrh	r3, [r3, #2]
 8001408:	2b00      	cmp	r3, #0
 800140a:	d11b      	bne.n	8001444 <MASS_NoData_Setup+0x60>
      && (pInformation->USBwIndex == 0) && (pInformation->USBwLength == 0x00))
 800140c:	4b11      	ldr	r3, [pc, #68]	; (8001454 <MASS_NoData_Setup+0x70>)
 800140e:	681b      	ldr	r3, [r3, #0]
 8001410:	889b      	ldrh	r3, [r3, #4]
 8001412:	2b00      	cmp	r3, #0
 8001414:	d116      	bne.n	8001444 <MASS_NoData_Setup+0x60>
 8001416:	4b0f      	ldr	r3, [pc, #60]	; (8001454 <MASS_NoData_Setup+0x70>)
 8001418:	681b      	ldr	r3, [r3, #0]
 800141a:	88db      	ldrh	r3, [r3, #6]
 800141c:	2b00      	cmp	r3, #0
 800141e:	d111      	bne.n	8001444 <MASS_NoData_Setup+0x60>
  
    /* Init EP2 OUT as Bulk endpoint */
    OTG_DEV_EP_Init(EP2_OUT, OTG_DEV_EP_TYPE_BULK, BULK_MAX_PACKET_SIZE);     
   #else
    /* Initialize Endpoint 1 */
    ClearDTOG_TX(ENDP1);
 8001420:	f04f 0001 	mov.w	r0, #1
 8001424:	f004 ff08 	bl	8006238 <ClearDTOG_TX>

    /* Initialize Endpoint 2 */
    ClearDTOG_RX(ENDP2);
 8001428:	f04f 0002 	mov.w	r0, #2
 800142c:	f004 fef0 	bl	8006210 <ClearDTOG_RX>
   #endif /* STM32F10X_CL */

    /*initialize the CBW signature to enable the clear feature*/
    CBW.dSignature = BOT_CBW_SIGNATURE;
 8001430:	4b09      	ldr	r3, [pc, #36]	; (8001458 <MASS_NoData_Setup+0x74>)
 8001432:	4a0a      	ldr	r2, [pc, #40]	; (800145c <MASS_NoData_Setup+0x78>)
 8001434:	601a      	str	r2, [r3, #0]
    Bot_State = BOT_IDLE;
 8001436:	4b0a      	ldr	r3, [pc, #40]	; (8001460 <MASS_NoData_Setup+0x7c>)
 8001438:	f04f 0200 	mov.w	r2, #0
 800143c:	701a      	strb	r2, [r3, #0]

    return USB_SUCCESS;
 800143e:	f04f 0300 	mov.w	r3, #0
 8001442:	e001      	b.n	8001448 <MASS_NoData_Setup+0x64>
  }
  return USB_UNSUPPORT;
 8001444:	f04f 0302 	mov.w	r3, #2
}
 8001448:	4618      	mov	r0, r3
 800144a:	f107 0708 	add.w	r7, r7, #8
 800144e:	46bd      	mov	sp, r7
 8001450:	bd80      	pop	{r7, pc}
 8001452:	bf00      	nop
 8001454:	2000050c 	.word	0x2000050c
 8001458:	20000458 	.word	0x20000458
 800145c:	43425355 	.word	0x43425355
 8001460:	2000047a 	.word	0x2000047a

08001464 <MASS_Get_Interface_Setting>:
* Input          : uint8_t Interface, uint8_t AlternateSetting.
* Output         : None.
* Return         : RESULT.
*******************************************************************************/
RESULT MASS_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting)
{
 8001464:	b480      	push	{r7}
 8001466:	b083      	sub	sp, #12
 8001468:	af00      	add	r7, sp, #0
 800146a:	4602      	mov	r2, r0
 800146c:	460b      	mov	r3, r1
 800146e:	71fa      	strb	r2, [r7, #7]
 8001470:	71bb      	strb	r3, [r7, #6]
  if (AlternateSetting > 0)
 8001472:	79bb      	ldrb	r3, [r7, #6]
 8001474:	2b00      	cmp	r3, #0
 8001476:	d002      	beq.n	800147e <MASS_Get_Interface_Setting+0x1a>
  {
    return USB_UNSUPPORT;/* in this application we don't have AlternateSetting*/
 8001478:	f04f 0302 	mov.w	r3, #2
 800147c:	e007      	b.n	800148e <MASS_Get_Interface_Setting+0x2a>
  }
  else if (Interface > 0)
 800147e:	79fb      	ldrb	r3, [r7, #7]
 8001480:	2b00      	cmp	r3, #0
 8001482:	d002      	beq.n	800148a <MASS_Get_Interface_Setting+0x26>
  {
    return USB_UNSUPPORT;/*in this application we have only 1 interfaces*/
 8001484:	f04f 0302 	mov.w	r3, #2
 8001488:	e001      	b.n	800148e <MASS_Get_Interface_Setting+0x2a>
  }
  return USB_SUCCESS;
 800148a:	f04f 0300 	mov.w	r3, #0
}
 800148e:	4618      	mov	r0, r3
 8001490:	f107 070c 	add.w	r7, r7, #12
 8001494:	46bd      	mov	sp, r7
 8001496:	bc80      	pop	{r7}
 8001498:	4770      	bx	lr
 800149a:	bf00      	nop

0800149c <MASS_GetDeviceDescriptor>:
* Input          : uint16_t Length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t *MASS_GetDeviceDescriptor(uint16_t Length)
{
 800149c:	b580      	push	{r7, lr}
 800149e:	b082      	sub	sp, #8
 80014a0:	af00      	add	r7, sp, #0
 80014a2:	4603      	mov	r3, r0
 80014a4:	80fb      	strh	r3, [r7, #6]
  return Standard_GetDescriptorData(Length, &Device_Descriptor );
 80014a6:	88fb      	ldrh	r3, [r7, #6]
 80014a8:	4618      	mov	r0, r3
 80014aa:	4904      	ldr	r1, [pc, #16]	; (80014bc <MASS_GetDeviceDescriptor+0x20>)
 80014ac:	f004 f84c 	bl	8005548 <Standard_GetDescriptorData>
 80014b0:	4603      	mov	r3, r0
}
 80014b2:	4618      	mov	r0, r3
 80014b4:	f107 0708 	add.w	r7, r7, #8
 80014b8:	46bd      	mov	sp, r7
 80014ba:	bd80      	pop	{r7, pc}
 80014bc:	20000124 	.word	0x20000124

080014c0 <MASS_GetConfigDescriptor>:
* Input          : uint16_t Length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t *MASS_GetConfigDescriptor(uint16_t Length)
{
 80014c0:	b580      	push	{r7, lr}
 80014c2:	b082      	sub	sp, #8
 80014c4:	af00      	add	r7, sp, #0
 80014c6:	4603      	mov	r3, r0
 80014c8:	80fb      	strh	r3, [r7, #6]
  return Standard_GetDescriptorData(Length, &Config_Descriptor );
 80014ca:	88fb      	ldrh	r3, [r7, #6]
 80014cc:	4618      	mov	r0, r3
 80014ce:	4904      	ldr	r1, [pc, #16]	; (80014e0 <MASS_GetConfigDescriptor+0x20>)
 80014d0:	f004 f83a 	bl	8005548 <Standard_GetDescriptorData>
 80014d4:	4603      	mov	r3, r0
}
 80014d6:	4618      	mov	r0, r3
 80014d8:	f107 0708 	add.w	r7, r7, #8
 80014dc:	46bd      	mov	sp, r7
 80014de:	bd80      	pop	{r7, pc}
 80014e0:	2000012c 	.word	0x2000012c

080014e4 <MASS_GetStringDescriptor>:
* Input          : uint16_t Length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t *MASS_GetStringDescriptor(uint16_t Length)
{
 80014e4:	b580      	push	{r7, lr}
 80014e6:	b084      	sub	sp, #16
 80014e8:	af00      	add	r7, sp, #0
 80014ea:	4603      	mov	r3, r0
 80014ec:	80fb      	strh	r3, [r7, #6]
  uint8_t wValue0 = pInformation->USBwValue0;
 80014ee:	4b0d      	ldr	r3, [pc, #52]	; (8001524 <MASS_GetStringDescriptor+0x40>)
 80014f0:	681b      	ldr	r3, [r3, #0]
 80014f2:	78db      	ldrb	r3, [r3, #3]
 80014f4:	73fb      	strb	r3, [r7, #15]

  if (wValue0 > 5)
 80014f6:	7bfb      	ldrb	r3, [r7, #15]
 80014f8:	2b05      	cmp	r3, #5
 80014fa:	d902      	bls.n	8001502 <MASS_GetStringDescriptor+0x1e>
  {
    return NULL;
 80014fc:	f04f 0300 	mov.w	r3, #0
 8001500:	e00a      	b.n	8001518 <MASS_GetStringDescriptor+0x34>
  }
  else
  {
    return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
 8001502:	7bfb      	ldrb	r3, [r7, #15]
 8001504:	ea4f 02c3 	mov.w	r2, r3, lsl #3
 8001508:	4b07      	ldr	r3, [pc, #28]	; (8001528 <MASS_GetStringDescriptor+0x44>)
 800150a:	18d3      	adds	r3, r2, r3
 800150c:	88fa      	ldrh	r2, [r7, #6]
 800150e:	4610      	mov	r0, r2
 8001510:	4619      	mov	r1, r3
 8001512:	f004 f819 	bl	8005548 <Standard_GetDescriptorData>
 8001516:	4603      	mov	r3, r0
  }
}
 8001518:	4618      	mov	r0, r3
 800151a:	f107 0710 	add.w	r7, r7, #16
 800151e:	46bd      	mov	sp, r7
 8001520:	bd80      	pop	{r7, pc}
 8001522:	bf00      	nop
 8001524:	2000050c 	.word	0x2000050c
 8001528:	20000134 	.word	0x20000134

0800152c <Get_Max_Lun>:
* Input          : uint16_t Length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t *Get_Max_Lun(uint16_t Length)
{
 800152c:	b480      	push	{r7}
 800152e:	b083      	sub	sp, #12
 8001530:	af00      	add	r7, sp, #0
 8001532:	4603      	mov	r3, r0
 8001534:	80fb      	strh	r3, [r7, #6]
  if (Length == 0)
 8001536:	88fb      	ldrh	r3, [r7, #6]
 8001538:	2b00      	cmp	r3, #0
 800153a:	d107      	bne.n	800154c <Get_Max_Lun+0x20>
  {
    pInformation->Ctrl_Info.Usb_wLength = LUN_DATA_LENGTH;
 800153c:	4b07      	ldr	r3, [pc, #28]	; (800155c <Get_Max_Lun+0x30>)
 800153e:	681b      	ldr	r3, [r3, #0]
 8001540:	f04f 0201 	mov.w	r2, #1
 8001544:	821a      	strh	r2, [r3, #16]
    return 0;
 8001546:	f04f 0300 	mov.w	r3, #0
 800154a:	e000      	b.n	800154e <Get_Max_Lun+0x22>
  }
  else
  {
    return((uint8_t*)(&Max_Lun));
 800154c:	4b04      	ldr	r3, [pc, #16]	; (8001560 <Get_Max_Lun+0x34>)
  }
}
 800154e:	4618      	mov	r0, r3
 8001550:	f107 070c 	add.w	r7, r7, #12
 8001554:	46bd      	mov	sp, r7
 8001556:	bc80      	pop	{r7}
 8001558:	4770      	bx	lr
 800155a:	bf00      	nop
 800155c:	2000050c 	.word	0x2000050c
 8001560:	200001a8 	.word	0x200001a8

08001564 <PowerOn>:
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
RESULT PowerOn(void)
{
 8001564:	b580      	push	{r7, lr}
 8001566:	b082      	sub	sp, #8
 8001568:	af00      	add	r7, sp, #0
#ifndef STM32F10X_CL
  uint16_t wRegVal;
  
  /*** cable plugged-in ? ***/
  /*while(!CablePluggedIn());*/
  USB_Cable_Config(ENABLE);
 800156a:	f04f 0001 	mov.w	r0, #1
 800156e:	f7fe fef1 	bl	8000354 <USB_Cable_Config>

  /*** CNTR_PWDN = 0 ***/
  wRegVal = CNTR_FRES;
 8001572:	f04f 0301 	mov.w	r3, #1
 8001576:	80fb      	strh	r3, [r7, #6]
  _SetCNTR(wRegVal);
 8001578:	4b0e      	ldr	r3, [pc, #56]	; (80015b4 <PowerOn+0x50>)
 800157a:	88fa      	ldrh	r2, [r7, #6]
 800157c:	601a      	str	r2, [r3, #0]

  /*** CNTR_FRES = 0 ***/
  wInterrupt_Mask = 0;
 800157e:	4b0e      	ldr	r3, [pc, #56]	; (80015b8 <PowerOn+0x54>)
 8001580:	f04f 0200 	mov.w	r2, #0
 8001584:	801a      	strh	r2, [r3, #0]
  _SetCNTR(wInterrupt_Mask);
 8001586:	4b0b      	ldr	r3, [pc, #44]	; (80015b4 <PowerOn+0x50>)
 8001588:	4a0b      	ldr	r2, [pc, #44]	; (80015b8 <PowerOn+0x54>)
 800158a:	8812      	ldrh	r2, [r2, #0]
 800158c:	601a      	str	r2, [r3, #0]
  /*** Clear pending interrupts ***/
  _SetISTR(0);
 800158e:	4b0b      	ldr	r3, [pc, #44]	; (80015bc <PowerOn+0x58>)
 8001590:	f04f 0200 	mov.w	r2, #0
 8001594:	601a      	str	r2, [r3, #0]
  /*** Set interrupt mask ***/
  wInterrupt_Mask = CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;
 8001596:	4b08      	ldr	r3, [pc, #32]	; (80015b8 <PowerOn+0x54>)
 8001598:	f44f 52e0 	mov.w	r2, #7168	; 0x1c00
 800159c:	801a      	strh	r2, [r3, #0]
  _SetCNTR(wInterrupt_Mask);
 800159e:	4b05      	ldr	r3, [pc, #20]	; (80015b4 <PowerOn+0x50>)
 80015a0:	4a05      	ldr	r2, [pc, #20]	; (80015b8 <PowerOn+0x54>)
 80015a2:	8812      	ldrh	r2, [r2, #0]
 80015a4:	601a      	str	r2, [r3, #0]
#endif /* STM32F10X_CL */
  
  return USB_SUCCESS;
 80015a6:	f04f 0300 	mov.w	r3, #0
}
 80015aa:	4618      	mov	r0, r3
 80015ac:	f107 0708 	add.w	r7, r7, #8
 80015b0:	46bd      	mov	sp, r7
 80015b2:	bd80      	pop	{r7, pc}
 80015b4:	40005c40 	.word	0x40005c40
 80015b8:	20000510 	.word	0x20000510
 80015bc:	40005c44 	.word	0x40005c44

080015c0 <PowerOff>:
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
RESULT PowerOff()
{
 80015c0:	b580      	push	{r7, lr}
 80015c2:	af00      	add	r7, sp, #0
#ifndef STM32F10X_CL  
  /* disable all interrupts and force USB reset */
  _SetCNTR(CNTR_FRES);
 80015c4:	4b09      	ldr	r3, [pc, #36]	; (80015ec <PowerOff+0x2c>)
 80015c6:	f04f 0201 	mov.w	r2, #1
 80015ca:	601a      	str	r2, [r3, #0]
  /* clear interrupt status register */
  _SetISTR(0);
 80015cc:	4b08      	ldr	r3, [pc, #32]	; (80015f0 <PowerOff+0x30>)
 80015ce:	f04f 0200 	mov.w	r2, #0
 80015d2:	601a      	str	r2, [r3, #0]
  /* Disable the Pull-Up*/
  USB_Cable_Config(DISABLE);
 80015d4:	f04f 0000 	mov.w	r0, #0
 80015d8:	f7fe febc 	bl	8000354 <USB_Cable_Config>
  /* switch-off device */
  _SetCNTR(CNTR_FRES + CNTR_PDWN);
 80015dc:	4b03      	ldr	r3, [pc, #12]	; (80015ec <PowerOff+0x2c>)
 80015de:	f04f 0203 	mov.w	r2, #3
 80015e2:	601a      	str	r2, [r3, #0]
  /* sw variables reset */
  /* ... */
#endif /* STM32F10X_CL */

  return USB_SUCCESS;
 80015e4:	f04f 0300 	mov.w	r3, #0
}
 80015e8:	4618      	mov	r0, r3
 80015ea:	bd80      	pop	{r7, pc}
 80015ec:	40005c40 	.word	0x40005c40
 80015f0:	40005c44 	.word	0x40005c44

080015f4 <Suspend>:
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
void Suspend(void)
{
 80015f4:	b580      	push	{r7, lr}
 80015f6:	b082      	sub	sp, #8
 80015f8:	af00      	add	r7, sp, #0
  
#ifndef STM32F10X_CL
  uint16_t wCNTR;

  /* macrocell enters suspend mode */
  wCNTR = _GetCNTR();
 80015fa:	4b0d      	ldr	r3, [pc, #52]	; (8001630 <Suspend+0x3c>)
 80015fc:	681b      	ldr	r3, [r3, #0]
 80015fe:	80fb      	strh	r3, [r7, #6]
  wCNTR |= CNTR_FSUSP;
 8001600:	88fb      	ldrh	r3, [r7, #6]
 8001602:	f043 0308 	orr.w	r3, r3, #8
 8001606:	80fb      	strh	r3, [r7, #6]
  _SetCNTR(wCNTR);
 8001608:	4b09      	ldr	r3, [pc, #36]	; (8001630 <Suspend+0x3c>)
 800160a:	88fa      	ldrh	r2, [r7, #6]
 800160c:	601a      	str	r2, [r3, #0]
  /* power reduction */
  /* ... on connected devices */

#ifndef STM32F10X_CL
  /* force low-power mode in the macrocell */
  wCNTR = _GetCNTR();
 800160e:	4b08      	ldr	r3, [pc, #32]	; (8001630 <Suspend+0x3c>)
 8001610:	681b      	ldr	r3, [r3, #0]
 8001612:	80fb      	strh	r3, [r7, #6]
  wCNTR |= CNTR_LPMODE;
 8001614:	88fb      	ldrh	r3, [r7, #6]
 8001616:	f043 0304 	orr.w	r3, r3, #4
 800161a:	80fb      	strh	r3, [r7, #6]
  _SetCNTR(wCNTR);
 800161c:	4b04      	ldr	r3, [pc, #16]	; (8001630 <Suspend+0x3c>)
 800161e:	88fa      	ldrh	r2, [r7, #6]
 8001620:	601a      	str	r2, [r3, #0]
#endif /* STM32F10X_CL */

  /* switch-off the clocks */
  /* ... */
  Enter_LowPowerMode();
 8001622:	f7fe fe11 	bl	8000248 <Enter_LowPowerMode>
}
 8001626:	f107 0708 	add.w	r7, r7, #8
 800162a:	46bd      	mov	sp, r7
 800162c:	bd80      	pop	{r7, pc}
 800162e:	bf00      	nop
 8001630:	40005c40 	.word	0x40005c40

08001634 <Resume_Init>:
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
void Resume_Init(void)
{
 8001634:	b580      	push	{r7, lr}
 8001636:	b082      	sub	sp, #8
 8001638:	af00      	add	r7, sp, #0
  /* ...  */
#ifndef STM32F10X_CL
  uint16_t wCNTR;
  
  /* CNTR_LPMODE = 0 */
  wCNTR = _GetCNTR();
 800163a:	4b0a      	ldr	r3, [pc, #40]	; (8001664 <Resume_Init+0x30>)
 800163c:	681b      	ldr	r3, [r3, #0]
 800163e:	80fb      	strh	r3, [r7, #6]
  wCNTR &= (~CNTR_LPMODE);
 8001640:	88fb      	ldrh	r3, [r7, #6]
 8001642:	f023 0304 	bic.w	r3, r3, #4
 8001646:	80fb      	strh	r3, [r7, #6]
  _SetCNTR(wCNTR);
 8001648:	4b06      	ldr	r3, [pc, #24]	; (8001664 <Resume_Init+0x30>)
 800164a:	88fa      	ldrh	r2, [r7, #6]
 800164c:	601a      	str	r2, [r3, #0]
#endif /* STM32F10X_CL */
  
  /* restore full power */
  /* ... on connected devices */
  Leave_LowPowerMode();
 800164e:	f7fe fe07 	bl	8000260 <Leave_LowPowerMode>

#ifndef STM32F10X_CL
  /* reset FSUSP bit */
  _SetCNTR(IMR_MSK);
 8001652:	4b04      	ldr	r3, [pc, #16]	; (8001664 <Resume_Init+0x30>)
 8001654:	f44f 4204 	mov.w	r2, #33792	; 0x8400
 8001658:	601a      	str	r2, [r3, #0]
#endif /* STM32F10X_CL */
  
  /* reverse suspend preparation */
  /* ... */
}
 800165a:	f107 0708 	add.w	r7, r7, #8
 800165e:	46bd      	mov	sp, r7
 8001660:	bd80      	pop	{r7, pc}
 8001662:	bf00      	nop
 8001664:	40005c40 	.word	0x40005c40

08001668 <Resume>:
*                  decrementing of the ESOF counter in different states.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Resume(RESUME_STATE eResumeSetVal)
{
 8001668:	b580      	push	{r7, lr}
 800166a:	b084      	sub	sp, #16
 800166c:	af00      	add	r7, sp, #0
 800166e:	4603      	mov	r3, r0
 8001670:	71fb      	strb	r3, [r7, #7]
#ifndef STM32F10X_CL
  uint16_t wCNTR;
#endif /* STM32F10X_CL */

  if (eResumeSetVal != RESUME_ESOF)
 8001672:	79fb      	ldrb	r3, [r7, #7]
 8001674:	2b07      	cmp	r3, #7
 8001676:	d002      	beq.n	800167e <Resume+0x16>
    ResumeS.eState = eResumeSetVal;
 8001678:	4b3e      	ldr	r3, [pc, #248]	; (8001774 <Resume+0x10c>)
 800167a:	79fa      	ldrb	r2, [r7, #7]
 800167c:	701a      	strb	r2, [r3, #0]

  switch (ResumeS.eState)
 800167e:	4b3d      	ldr	r3, [pc, #244]	; (8001774 <Resume+0x10c>)
 8001680:	781b      	ldrb	r3, [r3, #0]
 8001682:	b2db      	uxtb	r3, r3
 8001684:	2b05      	cmp	r3, #5
 8001686:	d867      	bhi.n	8001758 <Resume+0xf0>
 8001688:	a201      	add	r2, pc, #4	; (adr r2, 8001690 <Resume+0x28>)
 800168a:	f852 f023 	ldr.w	pc, [r2, r3, lsl #2]
 800168e:	bf00      	nop
 8001690:	080016a9 	.word	0x080016a9
 8001694:	080016b7 	.word	0x080016b7
 8001698:	080016c5 	.word	0x080016c5
 800169c:	080016d7 	.word	0x080016d7
 80016a0:	080016fb 	.word	0x080016fb
 80016a4:	08001721 	.word	0x08001721
  {
    case RESUME_EXTERNAL:
      Resume_Init();
 80016a8:	f7ff ffc4 	bl	8001634 <Resume_Init>
      ResumeS.eState = RESUME_OFF;
 80016ac:	4b31      	ldr	r3, [pc, #196]	; (8001774 <Resume+0x10c>)
 80016ae:	f04f 0206 	mov.w	r2, #6
 80016b2:	701a      	strb	r2, [r3, #0]
      break;
 80016b4:	e059      	b.n	800176a <Resume+0x102>
    case RESUME_INTERNAL:
      Resume_Init();
 80016b6:	f7ff ffbd 	bl	8001634 <Resume_Init>
      ResumeS.eState = RESUME_START;
 80016ba:	4b2e      	ldr	r3, [pc, #184]	; (8001774 <Resume+0x10c>)
 80016bc:	f04f 0204 	mov.w	r2, #4
 80016c0:	701a      	strb	r2, [r3, #0]
      break;
 80016c2:	e052      	b.n	800176a <Resume+0x102>
    case RESUME_LATER:
      ResumeS.bESOFcnt = 2;
 80016c4:	4b2b      	ldr	r3, [pc, #172]	; (8001774 <Resume+0x10c>)
 80016c6:	f04f 0202 	mov.w	r2, #2
 80016ca:	705a      	strb	r2, [r3, #1]
      ResumeS.eState = RESUME_WAIT;
 80016cc:	4b29      	ldr	r3, [pc, #164]	; (8001774 <Resume+0x10c>)
 80016ce:	f04f 0203 	mov.w	r2, #3
 80016d2:	701a      	strb	r2, [r3, #0]
      break;
 80016d4:	e049      	b.n	800176a <Resume+0x102>
    case RESUME_WAIT:
      ResumeS.bESOFcnt--;
 80016d6:	4b27      	ldr	r3, [pc, #156]	; (8001774 <Resume+0x10c>)
 80016d8:	785b      	ldrb	r3, [r3, #1]
 80016da:	b2db      	uxtb	r3, r3
 80016dc:	f103 33ff 	add.w	r3, r3, #4294967295
 80016e0:	b2da      	uxtb	r2, r3
 80016e2:	4b24      	ldr	r3, [pc, #144]	; (8001774 <Resume+0x10c>)
 80016e4:	705a      	strb	r2, [r3, #1]
      if (ResumeS.bESOFcnt == 0)
 80016e6:	4b23      	ldr	r3, [pc, #140]	; (8001774 <Resume+0x10c>)
 80016e8:	785b      	ldrb	r3, [r3, #1]
 80016ea:	b2db      	uxtb	r3, r3
 80016ec:	2b00      	cmp	r3, #0
 80016ee:	d139      	bne.n	8001764 <Resume+0xfc>
        ResumeS.eState = RESUME_START;
 80016f0:	4b20      	ldr	r3, [pc, #128]	; (8001774 <Resume+0x10c>)
 80016f2:	f04f 0204 	mov.w	r2, #4
 80016f6:	701a      	strb	r2, [r3, #0]
      break;
 80016f8:	e037      	b.n	800176a <Resume+0x102>
    case RESUME_START:
     #ifdef STM32F10X_CL
      OTGD_FS_SetRemoteWakeup();
     #else 
      wCNTR = _GetCNTR();
 80016fa:	4b1f      	ldr	r3, [pc, #124]	; (8001778 <Resume+0x110>)
 80016fc:	681b      	ldr	r3, [r3, #0]
 80016fe:	81fb      	strh	r3, [r7, #14]
      wCNTR |= CNTR_RESUME;
 8001700:	89fb      	ldrh	r3, [r7, #14]
 8001702:	f043 0310 	orr.w	r3, r3, #16
 8001706:	81fb      	strh	r3, [r7, #14]
      _SetCNTR(wCNTR);
 8001708:	4b1b      	ldr	r3, [pc, #108]	; (8001778 <Resume+0x110>)
 800170a:	89fa      	ldrh	r2, [r7, #14]
 800170c:	601a      	str	r2, [r3, #0]
     #endif /* STM32F10X_CL */
      ResumeS.eState = RESUME_ON;
 800170e:	4b19      	ldr	r3, [pc, #100]	; (8001774 <Resume+0x10c>)
 8001710:	f04f 0205 	mov.w	r2, #5
 8001714:	701a      	strb	r2, [r3, #0]
      ResumeS.bESOFcnt = 10;
 8001716:	4b17      	ldr	r3, [pc, #92]	; (8001774 <Resume+0x10c>)
 8001718:	f04f 020a 	mov.w	r2, #10
 800171c:	705a      	strb	r2, [r3, #1]
      break;
 800171e:	e024      	b.n	800176a <Resume+0x102>
    case RESUME_ON:
    #ifndef STM32F10X_CL      
      ResumeS.bESOFcnt--;
 8001720:	4b14      	ldr	r3, [pc, #80]	; (8001774 <Resume+0x10c>)
 8001722:	785b      	ldrb	r3, [r3, #1]
 8001724:	b2db      	uxtb	r3, r3
 8001726:	f103 33ff 	add.w	r3, r3, #4294967295
 800172a:	b2da      	uxtb	r2, r3
 800172c:	4b11      	ldr	r3, [pc, #68]	; (8001774 <Resume+0x10c>)
 800172e:	705a      	strb	r2, [r3, #1]
      if (ResumeS.bESOFcnt == 0)
 8001730:	4b10      	ldr	r3, [pc, #64]	; (8001774 <Resume+0x10c>)
 8001732:	785b      	ldrb	r3, [r3, #1]
 8001734:	b2db      	uxtb	r3, r3
 8001736:	2b00      	cmp	r3, #0
 8001738:	d116      	bne.n	8001768 <Resume+0x100>
      {
     #endif /* STM32F10X_CL */    
       #ifdef STM32F10X_CL
        OTGD_FS_ResetRemoteWakeup();
       #else
        wCNTR = _GetCNTR();
 800173a:	4b0f      	ldr	r3, [pc, #60]	; (8001778 <Resume+0x110>)
 800173c:	681b      	ldr	r3, [r3, #0]
 800173e:	81fb      	strh	r3, [r7, #14]
        wCNTR &= (~CNTR_RESUME);
 8001740:	89fb      	ldrh	r3, [r7, #14]
 8001742:	f023 0310 	bic.w	r3, r3, #16
 8001746:	81fb      	strh	r3, [r7, #14]
        _SetCNTR(wCNTR);
 8001748:	4b0b      	ldr	r3, [pc, #44]	; (8001778 <Resume+0x110>)
 800174a:	89fa      	ldrh	r2, [r7, #14]
 800174c:	601a      	str	r2, [r3, #0]
       #endif /* STM32F10X_CL */
        ResumeS.eState = RESUME_OFF;
 800174e:	4b09      	ldr	r3, [pc, #36]	; (8001774 <Resume+0x10c>)
 8001750:	f04f 0206 	mov.w	r2, #6
 8001754:	701a      	strb	r2, [r3, #0]
     #ifndef STM32F10X_CL
      }
     #endif /* STM32F10X_CL */
      break;
 8001756:	e008      	b.n	800176a <Resume+0x102>
    case RESUME_OFF:
    case RESUME_ESOF:
    default:
      ResumeS.eState = RESUME_OFF;
 8001758:	4b06      	ldr	r3, [pc, #24]	; (8001774 <Resume+0x10c>)
 800175a:	f04f 0206 	mov.w	r2, #6
 800175e:	701a      	strb	r2, [r3, #0]
      break;
 8001760:	bf00      	nop
 8001762:	e002      	b.n	800176a <Resume+0x102>
      break;
    case RESUME_WAIT:
      ResumeS.bESOFcnt--;
      if (ResumeS.bESOFcnt == 0)
        ResumeS.eState = RESUME_START;
      break;
 8001764:	bf00      	nop
 8001766:	e000      	b.n	800176a <Resume+0x102>
       #endif /* STM32F10X_CL */
        ResumeS.eState = RESUME_OFF;
     #ifndef STM32F10X_CL
      }
     #endif /* STM32F10X_CL */
      break;
 8001768:	bf00      	nop
    case RESUME_ESOF:
    default:
      ResumeS.eState = RESUME_OFF;
      break;
  }
}
 800176a:	f107 0710 	add.w	r7, r7, #16
 800176e:	46bd      	mov	sp, r7
 8001770:	bd80      	pop	{r7, pc}
 8001772:	bf00      	nop
 8001774:	200004d8 	.word	0x200004d8
 8001778:	40005c40 	.word	0x40005c40

0800177c <SCSI_Inquiry_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Inquiry_Cmd(uint8_t lun)
{
 800177c:	b580      	push	{r7, lr}
 800177e:	b084      	sub	sp, #16
 8001780:	af00      	add	r7, sp, #0
 8001782:	4603      	mov	r3, r0
 8001784:	71fb      	strb	r3, [r7, #7]
  uint8_t* Inquiry_Data;
  uint16_t Inquiry_Data_Length;

  if (CBW.CB[1] & 0x01)/*Evpd is set*/
 8001786:	4b14      	ldr	r3, [pc, #80]	; (80017d8 <SCSI_Inquiry_Cmd+0x5c>)
 8001788:	7c1b      	ldrb	r3, [r3, #16]
 800178a:	f003 0301 	and.w	r3, r3, #1
 800178e:	b2db      	uxtb	r3, r3
 8001790:	2b00      	cmp	r3, #0
 8001792:	d005      	beq.n	80017a0 <SCSI_Inquiry_Cmd+0x24>
  {
    Inquiry_Data = Page00_Inquiry_Data;
 8001794:	4b11      	ldr	r3, [pc, #68]	; (80017dc <SCSI_Inquiry_Cmd+0x60>)
 8001796:	60fb      	str	r3, [r7, #12]
    Inquiry_Data_Length = 5;
 8001798:	f04f 0305 	mov.w	r3, #5
 800179c:	817b      	strh	r3, [r7, #10]
 800179e:	e012      	b.n	80017c6 <SCSI_Inquiry_Cmd+0x4a>
  }
  else
  {

    if ( lun == 0)
 80017a0:	79fb      	ldrb	r3, [r7, #7]
 80017a2:	2b00      	cmp	r3, #0
 80017a4:	d102      	bne.n	80017ac <SCSI_Inquiry_Cmd+0x30>
    {
      Inquiry_Data = Standard_Inquiry_Data;
 80017a6:	4b0e      	ldr	r3, [pc, #56]	; (80017e0 <SCSI_Inquiry_Cmd+0x64>)
 80017a8:	60fb      	str	r3, [r7, #12]
 80017aa:	e001      	b.n	80017b0 <SCSI_Inquiry_Cmd+0x34>
    }
    else
    {
      Inquiry_Data = Standard_Inquiry_Data2;
 80017ac:	4b0d      	ldr	r3, [pc, #52]	; (80017e4 <SCSI_Inquiry_Cmd+0x68>)
 80017ae:	60fb      	str	r3, [r7, #12]
    }

    if (CBW.CB[4] <= STANDARD_INQUIRY_DATA_LEN)
 80017b0:	4b09      	ldr	r3, [pc, #36]	; (80017d8 <SCSI_Inquiry_Cmd+0x5c>)
 80017b2:	7cdb      	ldrb	r3, [r3, #19]
 80017b4:	2b24      	cmp	r3, #36	; 0x24
 80017b6:	d803      	bhi.n	80017c0 <SCSI_Inquiry_Cmd+0x44>
      Inquiry_Data_Length = CBW.CB[4];
 80017b8:	4b07      	ldr	r3, [pc, #28]	; (80017d8 <SCSI_Inquiry_Cmd+0x5c>)
 80017ba:	7cdb      	ldrb	r3, [r3, #19]
 80017bc:	817b      	strh	r3, [r7, #10]
 80017be:	e002      	b.n	80017c6 <SCSI_Inquiry_Cmd+0x4a>
    else
      Inquiry_Data_Length = STANDARD_INQUIRY_DATA_LEN;
 80017c0:	f04f 0324 	mov.w	r3, #36	; 0x24
 80017c4:	817b      	strh	r3, [r7, #10]

  }
  Transfer_Data_Request(Inquiry_Data, Inquiry_Data_Length);
 80017c6:	897b      	ldrh	r3, [r7, #10]
 80017c8:	68f8      	ldr	r0, [r7, #12]
 80017ca:	4619      	mov	r1, r3
 80017cc:	f7ff fc06 	bl	8000fdc <Transfer_Data_Request>
}
 80017d0:	f107 0710 	add.w	r7, r7, #16
 80017d4:	46bd      	mov	sp, r7
 80017d6:	bd80      	pop	{r7, pc}
 80017d8:	20000458 	.word	0x20000458
 80017dc:	20000198 	.word	0x20000198
 80017e0:	20000004 	.word	0x20000004
 80017e4:	20000028 	.word	0x20000028

080017e8 <SCSI_ReadFormatCapacity_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_ReadFormatCapacity_Cmd(uint8_t lun)
{
 80017e8:	b580      	push	{r7, lr}
 80017ea:	b082      	sub	sp, #8
 80017ec:	af00      	add	r7, sp, #0
 80017ee:	4603      	mov	r3, r0
 80017f0:	71fb      	strb	r3, [r7, #7]

  if (MAL_GetStatus(lun) != 0 )
 80017f2:	79fb      	ldrb	r3, [r7, #7]
 80017f4:	4618      	mov	r0, r3
 80017f6:	f7fe fe97 	bl	8000528 <MAL_GetStatus>
 80017fa:	4603      	mov	r3, r0
 80017fc:	2b00      	cmp	r3, #0
 80017fe:	d013      	beq.n	8001828 <SCSI_ReadFormatCapacity_Cmd+0x40>
  {
    Set_Scsi_Sense_Data(CBW.bLUN, NOT_READY, MEDIUM_NOT_PRESENT);
 8001800:	4b2b      	ldr	r3, [pc, #172]	; (80018b0 <SCSI_ReadFormatCapacity_Cmd+0xc8>)
 8001802:	7b5b      	ldrb	r3, [r3, #13]
 8001804:	4618      	mov	r0, r3
 8001806:	f04f 0102 	mov.w	r1, #2
 800180a:	f04f 023a 	mov.w	r2, #58	; 0x3a
 800180e:	f000 f913 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_ENABLE);
 8001812:	f04f 0001 	mov.w	r0, #1
 8001816:	f04f 0101 	mov.w	r1, #1
 800181a:	f7ff fc09 	bl	8001030 <Set_CSW>
    Bot_Abort(DIR_IN);
 800181e:	f04f 0000 	mov.w	r0, #0
 8001822:	f7ff fc35 	bl	8001090 <Bot_Abort>
    return;
 8001826:	e03f      	b.n	80018a8 <SCSI_ReadFormatCapacity_Cmd+0xc0>
  }
  ReadFormatCapacity_Data[4] = (uint8_t)(Mass_Block_Count[lun] >> 24);
 8001828:	79fa      	ldrb	r2, [r7, #7]
 800182a:	4b22      	ldr	r3, [pc, #136]	; (80018b4 <SCSI_ReadFormatCapacity_Cmd+0xcc>)
 800182c:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001830:	ea4f 6313 	mov.w	r3, r3, lsr #24
 8001834:	b2da      	uxtb	r2, r3
 8001836:	4b20      	ldr	r3, [pc, #128]	; (80018b8 <SCSI_ReadFormatCapacity_Cmd+0xd0>)
 8001838:	711a      	strb	r2, [r3, #4]
  ReadFormatCapacity_Data[5] = (uint8_t)(Mass_Block_Count[lun] >> 16);
 800183a:	79fa      	ldrb	r2, [r7, #7]
 800183c:	4b1d      	ldr	r3, [pc, #116]	; (80018b4 <SCSI_ReadFormatCapacity_Cmd+0xcc>)
 800183e:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001842:	ea4f 4313 	mov.w	r3, r3, lsr #16
 8001846:	b2da      	uxtb	r2, r3
 8001848:	4b1b      	ldr	r3, [pc, #108]	; (80018b8 <SCSI_ReadFormatCapacity_Cmd+0xd0>)
 800184a:	715a      	strb	r2, [r3, #5]
  ReadFormatCapacity_Data[6] = (uint8_t)(Mass_Block_Count[lun] >>  8);
 800184c:	79fa      	ldrb	r2, [r7, #7]
 800184e:	4b19      	ldr	r3, [pc, #100]	; (80018b4 <SCSI_ReadFormatCapacity_Cmd+0xcc>)
 8001850:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001854:	ea4f 2313 	mov.w	r3, r3, lsr #8
 8001858:	b2da      	uxtb	r2, r3
 800185a:	4b17      	ldr	r3, [pc, #92]	; (80018b8 <SCSI_ReadFormatCapacity_Cmd+0xd0>)
 800185c:	719a      	strb	r2, [r3, #6]
  ReadFormatCapacity_Data[7] = (uint8_t)(Mass_Block_Count[lun]);
 800185e:	79fa      	ldrb	r2, [r7, #7]
 8001860:	4b14      	ldr	r3, [pc, #80]	; (80018b4 <SCSI_ReadFormatCapacity_Cmd+0xcc>)
 8001862:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001866:	b2da      	uxtb	r2, r3
 8001868:	4b13      	ldr	r3, [pc, #76]	; (80018b8 <SCSI_ReadFormatCapacity_Cmd+0xd0>)
 800186a:	71da      	strb	r2, [r3, #7]

  ReadFormatCapacity_Data[9] = (uint8_t)(Mass_Block_Size[lun] >>  16);
 800186c:	79fa      	ldrb	r2, [r7, #7]
 800186e:	4b13      	ldr	r3, [pc, #76]	; (80018bc <SCSI_ReadFormatCapacity_Cmd+0xd4>)
 8001870:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001874:	ea4f 4313 	mov.w	r3, r3, lsr #16
 8001878:	b2da      	uxtb	r2, r3
 800187a:	4b0f      	ldr	r3, [pc, #60]	; (80018b8 <SCSI_ReadFormatCapacity_Cmd+0xd0>)
 800187c:	725a      	strb	r2, [r3, #9]
  ReadFormatCapacity_Data[10] = (uint8_t)(Mass_Block_Size[lun] >>  8);
 800187e:	79fa      	ldrb	r2, [r7, #7]
 8001880:	4b0e      	ldr	r3, [pc, #56]	; (80018bc <SCSI_ReadFormatCapacity_Cmd+0xd4>)
 8001882:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001886:	ea4f 2313 	mov.w	r3, r3, lsr #8
 800188a:	b2da      	uxtb	r2, r3
 800188c:	4b0a      	ldr	r3, [pc, #40]	; (80018b8 <SCSI_ReadFormatCapacity_Cmd+0xd0>)
 800188e:	729a      	strb	r2, [r3, #10]
  ReadFormatCapacity_Data[11] = (uint8_t)(Mass_Block_Size[lun]);
 8001890:	79fa      	ldrb	r2, [r7, #7]
 8001892:	4b0a      	ldr	r3, [pc, #40]	; (80018bc <SCSI_ReadFormatCapacity_Cmd+0xd4>)
 8001894:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001898:	b2da      	uxtb	r2, r3
 800189a:	4b07      	ldr	r3, [pc, #28]	; (80018b8 <SCSI_ReadFormatCapacity_Cmd+0xd0>)
 800189c:	72da      	strb	r2, [r3, #11]
  Transfer_Data_Request(ReadFormatCapacity_Data, READ_FORMAT_CAPACITY_DATA_LEN);
 800189e:	4806      	ldr	r0, [pc, #24]	; (80018b8 <SCSI_ReadFormatCapacity_Cmd+0xd0>)
 80018a0:	f04f 010c 	mov.w	r1, #12
 80018a4:	f7ff fb9a 	bl	8000fdc <Transfer_Data_Request>
}
 80018a8:	f107 0708 	add.w	r7, r7, #8
 80018ac:	46bd      	mov	sp, r7
 80018ae:	bd80      	pop	{r7, pc}
 80018b0:	20000458 	.word	0x20000458
 80018b4:	20000248 	.word	0x20000248
 80018b8:	2000006c 	.word	0x2000006c
 80018bc:	20000240 	.word	0x20000240

080018c0 <SCSI_ReadCapacity10_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_ReadCapacity10_Cmd(uint8_t lun)
{
 80018c0:	b580      	push	{r7, lr}
 80018c2:	b082      	sub	sp, #8
 80018c4:	af00      	add	r7, sp, #0
 80018c6:	4603      	mov	r3, r0
 80018c8:	71fb      	strb	r3, [r7, #7]

  if (MAL_GetStatus(lun))
 80018ca:	79fb      	ldrb	r3, [r7, #7]
 80018cc:	4618      	mov	r0, r3
 80018ce:	f7fe fe2b 	bl	8000528 <MAL_GetStatus>
 80018d2:	4603      	mov	r3, r0
 80018d4:	2b00      	cmp	r3, #0
 80018d6:	d013      	beq.n	8001900 <SCSI_ReadCapacity10_Cmd+0x40>
  {
    Set_Scsi_Sense_Data(CBW.bLUN, NOT_READY, MEDIUM_NOT_PRESENT);
 80018d8:	4b34      	ldr	r3, [pc, #208]	; (80019ac <SCSI_ReadCapacity10_Cmd+0xec>)
 80018da:	7b5b      	ldrb	r3, [r3, #13]
 80018dc:	4618      	mov	r0, r3
 80018de:	f04f 0102 	mov.w	r1, #2
 80018e2:	f04f 023a 	mov.w	r2, #58	; 0x3a
 80018e6:	f000 f8a7 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_ENABLE);
 80018ea:	f04f 0001 	mov.w	r0, #1
 80018ee:	f04f 0101 	mov.w	r1, #1
 80018f2:	f7ff fb9d 	bl	8001030 <Set_CSW>
    Bot_Abort(DIR_IN);
 80018f6:	f04f 0000 	mov.w	r0, #0
 80018fa:	f7ff fbc9 	bl	8001090 <Bot_Abort>
    return;
 80018fe:	e051      	b.n	80019a4 <SCSI_ReadCapacity10_Cmd+0xe4>
  }

  ReadCapacity10_Data[0] = (uint8_t)((Mass_Block_Count[lun] - 1) >> 24);
 8001900:	79fa      	ldrb	r2, [r7, #7]
 8001902:	4b2b      	ldr	r3, [pc, #172]	; (80019b0 <SCSI_ReadCapacity10_Cmd+0xf0>)
 8001904:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001908:	f103 33ff 	add.w	r3, r3, #4294967295
 800190c:	ea4f 6313 	mov.w	r3, r3, lsr #24
 8001910:	b2da      	uxtb	r2, r3
 8001912:	4b28      	ldr	r3, [pc, #160]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 8001914:	701a      	strb	r2, [r3, #0]
  ReadCapacity10_Data[1] = (uint8_t)((Mass_Block_Count[lun] - 1) >> 16);
 8001916:	79fa      	ldrb	r2, [r7, #7]
 8001918:	4b25      	ldr	r3, [pc, #148]	; (80019b0 <SCSI_ReadCapacity10_Cmd+0xf0>)
 800191a:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 800191e:	f103 33ff 	add.w	r3, r3, #4294967295
 8001922:	ea4f 4313 	mov.w	r3, r3, lsr #16
 8001926:	b2da      	uxtb	r2, r3
 8001928:	4b22      	ldr	r3, [pc, #136]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 800192a:	705a      	strb	r2, [r3, #1]
  ReadCapacity10_Data[2] = (uint8_t)((Mass_Block_Count[lun] - 1) >>  8);
 800192c:	79fa      	ldrb	r2, [r7, #7]
 800192e:	4b20      	ldr	r3, [pc, #128]	; (80019b0 <SCSI_ReadCapacity10_Cmd+0xf0>)
 8001930:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001934:	f103 33ff 	add.w	r3, r3, #4294967295
 8001938:	ea4f 2313 	mov.w	r3, r3, lsr #8
 800193c:	b2da      	uxtb	r2, r3
 800193e:	4b1d      	ldr	r3, [pc, #116]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 8001940:	709a      	strb	r2, [r3, #2]
  ReadCapacity10_Data[3] = (uint8_t)(Mass_Block_Count[lun] - 1);
 8001942:	79fa      	ldrb	r2, [r7, #7]
 8001944:	4b1a      	ldr	r3, [pc, #104]	; (80019b0 <SCSI_ReadCapacity10_Cmd+0xf0>)
 8001946:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 800194a:	b2db      	uxtb	r3, r3
 800194c:	f103 33ff 	add.w	r3, r3, #4294967295
 8001950:	b2da      	uxtb	r2, r3
 8001952:	4b18      	ldr	r3, [pc, #96]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 8001954:	70da      	strb	r2, [r3, #3]

  ReadCapacity10_Data[4] = (uint8_t)(Mass_Block_Size[lun] >>  24);
 8001956:	79fa      	ldrb	r2, [r7, #7]
 8001958:	4b17      	ldr	r3, [pc, #92]	; (80019b8 <SCSI_ReadCapacity10_Cmd+0xf8>)
 800195a:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 800195e:	ea4f 6313 	mov.w	r3, r3, lsr #24
 8001962:	b2da      	uxtb	r2, r3
 8001964:	4b13      	ldr	r3, [pc, #76]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 8001966:	711a      	strb	r2, [r3, #4]
  ReadCapacity10_Data[5] = (uint8_t)(Mass_Block_Size[lun] >>  16);
 8001968:	79fa      	ldrb	r2, [r7, #7]
 800196a:	4b13      	ldr	r3, [pc, #76]	; (80019b8 <SCSI_ReadCapacity10_Cmd+0xf8>)
 800196c:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001970:	ea4f 4313 	mov.w	r3, r3, lsr #16
 8001974:	b2da      	uxtb	r2, r3
 8001976:	4b0f      	ldr	r3, [pc, #60]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 8001978:	715a      	strb	r2, [r3, #5]
  ReadCapacity10_Data[6] = (uint8_t)(Mass_Block_Size[lun] >>  8);
 800197a:	79fa      	ldrb	r2, [r7, #7]
 800197c:	4b0e      	ldr	r3, [pc, #56]	; (80019b8 <SCSI_ReadCapacity10_Cmd+0xf8>)
 800197e:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001982:	ea4f 2313 	mov.w	r3, r3, lsr #8
 8001986:	b2da      	uxtb	r2, r3
 8001988:	4b0a      	ldr	r3, [pc, #40]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 800198a:	719a      	strb	r2, [r3, #6]
  ReadCapacity10_Data[7] = (uint8_t)(Mass_Block_Size[lun]);
 800198c:	79fa      	ldrb	r2, [r7, #7]
 800198e:	4b0a      	ldr	r3, [pc, #40]	; (80019b8 <SCSI_ReadCapacity10_Cmd+0xf8>)
 8001990:	f853 3022 	ldr.w	r3, [r3, r2, lsl #2]
 8001994:	b2da      	uxtb	r2, r3
 8001996:	4b07      	ldr	r3, [pc, #28]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 8001998:	71da      	strb	r2, [r3, #7]
  Transfer_Data_Request(ReadCapacity10_Data, READ_CAPACITY10_DATA_LEN);
 800199a:	4806      	ldr	r0, [pc, #24]	; (80019b4 <SCSI_ReadCapacity10_Cmd+0xf4>)
 800199c:	f04f 0108 	mov.w	r1, #8
 80019a0:	f7ff fb1c 	bl	8000fdc <Transfer_Data_Request>
}
 80019a4:	f107 0708 	add.w	r7, r7, #8
 80019a8:	46bd      	mov	sp, r7
 80019aa:	bd80      	pop	{r7, pc}
 80019ac:	20000458 	.word	0x20000458
 80019b0:	20000248 	.word	0x20000248
 80019b4:	200001a0 	.word	0x200001a0
 80019b8:	20000240 	.word	0x20000240

080019bc <SCSI_ModeSense6_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_ModeSense6_Cmd (uint8_t lun)
{
 80019bc:	b580      	push	{r7, lr}
 80019be:	b082      	sub	sp, #8
 80019c0:	af00      	add	r7, sp, #0
 80019c2:	4603      	mov	r3, r0
 80019c4:	71fb      	strb	r3, [r7, #7]
  Transfer_Data_Request(Mode_Sense6_data, MODE_SENSE6_DATA_LEN);
 80019c6:	4804      	ldr	r0, [pc, #16]	; (80019d8 <SCSI_ModeSense6_Cmd+0x1c>)
 80019c8:	f04f 0104 	mov.w	r1, #4
 80019cc:	f7ff fb06 	bl	8000fdc <Transfer_Data_Request>
}
 80019d0:	f107 0708 	add.w	r7, r7, #8
 80019d4:	46bd      	mov	sp, r7
 80019d6:	bd80      	pop	{r7, pc}
 80019d8:	2000004c 	.word	0x2000004c

080019dc <SCSI_ModeSense10_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_ModeSense10_Cmd (uint8_t lun)
{
 80019dc:	b580      	push	{r7, lr}
 80019de:	b082      	sub	sp, #8
 80019e0:	af00      	add	r7, sp, #0
 80019e2:	4603      	mov	r3, r0
 80019e4:	71fb      	strb	r3, [r7, #7]
  Transfer_Data_Request(Mode_Sense10_data, MODE_SENSE10_DATA_LEN);
 80019e6:	4804      	ldr	r0, [pc, #16]	; (80019f8 <SCSI_ModeSense10_Cmd+0x1c>)
 80019e8:	f04f 0108 	mov.w	r1, #8
 80019ec:	f7ff faf6 	bl	8000fdc <Transfer_Data_Request>
}
 80019f0:	f107 0708 	add.w	r7, r7, #8
 80019f4:	46bd      	mov	sp, r7
 80019f6:	bd80      	pop	{r7, pc}
 80019f8:	20000050 	.word	0x20000050

080019fc <SCSI_RequestSense_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_RequestSense_Cmd (uint8_t lun)
{
 80019fc:	b580      	push	{r7, lr}
 80019fe:	b084      	sub	sp, #16
 8001a00:	af00      	add	r7, sp, #0
 8001a02:	4603      	mov	r3, r0
 8001a04:	71fb      	strb	r3, [r7, #7]
  uint8_t Request_Sense_data_Length;

  if (CBW.CB[4] <= REQUEST_SENSE_DATA_LEN)
 8001a06:	4b0a      	ldr	r3, [pc, #40]	; (8001a30 <SCSI_RequestSense_Cmd+0x34>)
 8001a08:	7cdb      	ldrb	r3, [r3, #19]
 8001a0a:	2b12      	cmp	r3, #18
 8001a0c:	d803      	bhi.n	8001a16 <SCSI_RequestSense_Cmd+0x1a>
  {
    Request_Sense_data_Length = CBW.CB[4];
 8001a0e:	4b08      	ldr	r3, [pc, #32]	; (8001a30 <SCSI_RequestSense_Cmd+0x34>)
 8001a10:	7cdb      	ldrb	r3, [r3, #19]
 8001a12:	73fb      	strb	r3, [r7, #15]
 8001a14:	e002      	b.n	8001a1c <SCSI_RequestSense_Cmd+0x20>
  }
  else
  {
    Request_Sense_data_Length = REQUEST_SENSE_DATA_LEN;
 8001a16:	f04f 0312 	mov.w	r3, #18
 8001a1a:	73fb      	strb	r3, [r7, #15]
  }
  Transfer_Data_Request(Scsi_Sense_Data, Request_Sense_data_Length);
 8001a1c:	7bfb      	ldrb	r3, [r7, #15]
 8001a1e:	b29b      	uxth	r3, r3
 8001a20:	4804      	ldr	r0, [pc, #16]	; (8001a34 <SCSI_RequestSense_Cmd+0x38>)
 8001a22:	4619      	mov	r1, r3
 8001a24:	f7ff fada 	bl	8000fdc <Transfer_Data_Request>
}
 8001a28:	f107 0710 	add.w	r7, r7, #16
 8001a2c:	46bd      	mov	sp, r7
 8001a2e:	bd80      	pop	{r7, pc}
 8001a30:	20000458 	.word	0x20000458
 8001a34:	20000058 	.word	0x20000058

08001a38 <Set_Scsi_Sense_Data>:
                   uint8_t Asc.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_Scsi_Sense_Data(uint8_t lun, uint8_t Sens_Key, uint8_t Asc)
{
 8001a38:	b480      	push	{r7}
 8001a3a:	b083      	sub	sp, #12
 8001a3c:	af00      	add	r7, sp, #0
 8001a3e:	4613      	mov	r3, r2
 8001a40:	4602      	mov	r2, r0
 8001a42:	71fa      	strb	r2, [r7, #7]
 8001a44:	460a      	mov	r2, r1
 8001a46:	71ba      	strb	r2, [r7, #6]
 8001a48:	717b      	strb	r3, [r7, #5]
  Scsi_Sense_Data[2] = Sens_Key;
 8001a4a:	4b05      	ldr	r3, [pc, #20]	; (8001a60 <Set_Scsi_Sense_Data+0x28>)
 8001a4c:	79ba      	ldrb	r2, [r7, #6]
 8001a4e:	709a      	strb	r2, [r3, #2]
  Scsi_Sense_Data[12] = Asc;
 8001a50:	4b03      	ldr	r3, [pc, #12]	; (8001a60 <Set_Scsi_Sense_Data+0x28>)
 8001a52:	797a      	ldrb	r2, [r7, #5]
 8001a54:	731a      	strb	r2, [r3, #12]
}
 8001a56:	f107 070c 	add.w	r7, r7, #12
 8001a5a:	46bd      	mov	sp, r7
 8001a5c:	bc80      	pop	{r7}
 8001a5e:	4770      	bx	lr
 8001a60:	20000058 	.word	0x20000058

08001a64 <SCSI_Start_Stop_Unit_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Start_Stop_Unit_Cmd(uint8_t lun)
{
 8001a64:	b580      	push	{r7, lr}
 8001a66:	b082      	sub	sp, #8
 8001a68:	af00      	add	r7, sp, #0
 8001a6a:	4603      	mov	r3, r0
 8001a6c:	71fb      	strb	r3, [r7, #7]
  Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
 8001a6e:	f04f 0000 	mov.w	r0, #0
 8001a72:	f04f 0101 	mov.w	r1, #1
 8001a76:	f7ff fadb 	bl	8001030 <Set_CSW>
}
 8001a7a:	f107 0708 	add.w	r7, r7, #8
 8001a7e:	46bd      	mov	sp, r7
 8001a80:	bd80      	pop	{r7, pc}
 8001a82:	bf00      	nop

08001a84 <SCSI_Read10_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Read10_Cmd(uint8_t lun , uint32_t LBA , uint32_t BlockNbr)
{
 8001a84:	b580      	push	{r7, lr}
 8001a86:	b084      	sub	sp, #16
 8001a88:	af00      	add	r7, sp, #0
 8001a8a:	4603      	mov	r3, r0
 8001a8c:	60b9      	str	r1, [r7, #8]
 8001a8e:	607a      	str	r2, [r7, #4]
 8001a90:	73fb      	strb	r3, [r7, #15]

  if (Bot_State == BOT_IDLE)
 8001a92:	4b22      	ldr	r3, [pc, #136]	; (8001b1c <SCSI_Read10_Cmd+0x98>)
 8001a94:	781b      	ldrb	r3, [r3, #0]
 8001a96:	2b00      	cmp	r3, #0
 8001a98:	d130      	bne.n	8001afc <SCSI_Read10_Cmd+0x78>
  {
    if (!(SCSI_Address_Management(CBW.bLUN, SCSI_READ10, LBA, BlockNbr)))/*address out of range*/
 8001a9a:	4b21      	ldr	r3, [pc, #132]	; (8001b20 <SCSI_Read10_Cmd+0x9c>)
 8001a9c:	7b5b      	ldrb	r3, [r3, #13]
 8001a9e:	4618      	mov	r0, r3
 8001aa0:	f04f 0128 	mov.w	r1, #40	; 0x28
 8001aa4:	68ba      	ldr	r2, [r7, #8]
 8001aa6:	687b      	ldr	r3, [r7, #4]
 8001aa8:	f000 f96a 	bl	8001d80 <SCSI_Address_Management>
 8001aac:	4603      	mov	r3, r0
 8001aae:	2b00      	cmp	r3, #0
 8001ab0:	d02f      	beq.n	8001b12 <SCSI_Read10_Cmd+0x8e>
    {
      return;
    }

    if ((CBW.bmFlags & 0x80) != 0)
 8001ab2:	4b1b      	ldr	r3, [pc, #108]	; (8001b20 <SCSI_Read10_Cmd+0x9c>)
 8001ab4:	7b1b      	ldrb	r3, [r3, #12]
 8001ab6:	b2db      	uxtb	r3, r3
 8001ab8:	b25b      	sxtb	r3, r3
 8001aba:	2b00      	cmp	r3, #0
 8001abc:	da0a      	bge.n	8001ad4 <SCSI_Read10_Cmd+0x50>
    {
      Bot_State = BOT_DATA_IN;
 8001abe:	4b17      	ldr	r3, [pc, #92]	; (8001b1c <SCSI_Read10_Cmd+0x98>)
 8001ac0:	f04f 0202 	mov.w	r2, #2
 8001ac4:	701a      	strb	r2, [r3, #0]
      Read_Memory(lun, LBA , BlockNbr);
 8001ac6:	7bfb      	ldrb	r3, [r7, #15]
 8001ac8:	4618      	mov	r0, r3
 8001aca:	68b9      	ldr	r1, [r7, #8]
 8001acc:	687a      	ldr	r2, [r7, #4]
 8001ace:	f7fe fd93 	bl	80005f8 <Read_Memory>
    {
      Bot_Abort(BOTH_DIR);
      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
      Set_CSW (CSW_CMD_FAILED, SEND_CSW_ENABLE);
    }
    return;
 8001ad2:	e01f      	b.n	8001b14 <SCSI_Read10_Cmd+0x90>
      Bot_State = BOT_DATA_IN;
      Read_Memory(lun, LBA , BlockNbr);
    }
    else
    {
      Bot_Abort(BOTH_DIR);
 8001ad4:	f04f 0002 	mov.w	r0, #2
 8001ad8:	f7ff fada 	bl	8001090 <Bot_Abort>
      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
 8001adc:	4b10      	ldr	r3, [pc, #64]	; (8001b20 <SCSI_Read10_Cmd+0x9c>)
 8001ade:	7b5b      	ldrb	r3, [r3, #13]
 8001ae0:	4618      	mov	r0, r3
 8001ae2:	f04f 0105 	mov.w	r1, #5
 8001ae6:	f04f 0224 	mov.w	r2, #36	; 0x24
 8001aea:	f7ff ffa5 	bl	8001a38 <Set_Scsi_Sense_Data>
      Set_CSW (CSW_CMD_FAILED, SEND_CSW_ENABLE);
 8001aee:	f04f 0001 	mov.w	r0, #1
 8001af2:	f04f 0101 	mov.w	r1, #1
 8001af6:	f7ff fa9b 	bl	8001030 <Set_CSW>
    }
    return;
 8001afa:	e00b      	b.n	8001b14 <SCSI_Read10_Cmd+0x90>
  }
  else if (Bot_State == BOT_DATA_IN)
 8001afc:	4b07      	ldr	r3, [pc, #28]	; (8001b1c <SCSI_Read10_Cmd+0x98>)
 8001afe:	781b      	ldrb	r3, [r3, #0]
 8001b00:	2b02      	cmp	r3, #2
 8001b02:	d107      	bne.n	8001b14 <SCSI_Read10_Cmd+0x90>
  {
    Read_Memory(lun , LBA , BlockNbr);
 8001b04:	7bfb      	ldrb	r3, [r7, #15]
 8001b06:	4618      	mov	r0, r3
 8001b08:	68b9      	ldr	r1, [r7, #8]
 8001b0a:	687a      	ldr	r2, [r7, #4]
 8001b0c:	f7fe fd74 	bl	80005f8 <Read_Memory>
 8001b10:	e000      	b.n	8001b14 <SCSI_Read10_Cmd+0x90>

  if (Bot_State == BOT_IDLE)
  {
    if (!(SCSI_Address_Management(CBW.bLUN, SCSI_READ10, LBA, BlockNbr)))/*address out of range*/
    {
      return;
 8001b12:	bf00      	nop
  }
  else if (Bot_State == BOT_DATA_IN)
  {
    Read_Memory(lun , LBA , BlockNbr);
  }
}
 8001b14:	f107 0710 	add.w	r7, r7, #16
 8001b18:	46bd      	mov	sp, r7
 8001b1a:	bd80      	pop	{r7, pc}
 8001b1c:	2000047a 	.word	0x2000047a
 8001b20:	20000458 	.word	0x20000458

08001b24 <SCSI_Write10_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Write10_Cmd(uint8_t lun , uint32_t LBA , uint32_t BlockNbr)
{
 8001b24:	b580      	push	{r7, lr}
 8001b26:	b084      	sub	sp, #16
 8001b28:	af00      	add	r7, sp, #0
 8001b2a:	4603      	mov	r3, r0
 8001b2c:	60b9      	str	r1, [r7, #8]
 8001b2e:	607a      	str	r2, [r7, #4]
 8001b30:	73fb      	strb	r3, [r7, #15]

  if (Bot_State == BOT_IDLE)
 8001b32:	4b22      	ldr	r3, [pc, #136]	; (8001bbc <SCSI_Write10_Cmd+0x98>)
 8001b34:	781b      	ldrb	r3, [r3, #0]
 8001b36:	2b00      	cmp	r3, #0
 8001b38:	d130      	bne.n	8001b9c <SCSI_Write10_Cmd+0x78>
  {
    if (!(SCSI_Address_Management(CBW.bLUN, SCSI_WRITE10 , LBA, BlockNbr)))/*address out of range*/
 8001b3a:	4b21      	ldr	r3, [pc, #132]	; (8001bc0 <SCSI_Write10_Cmd+0x9c>)
 8001b3c:	7b5b      	ldrb	r3, [r3, #13]
 8001b3e:	4618      	mov	r0, r3
 8001b40:	f04f 012a 	mov.w	r1, #42	; 0x2a
 8001b44:	68ba      	ldr	r2, [r7, #8]
 8001b46:	687b      	ldr	r3, [r7, #4]
 8001b48:	f000 f91a 	bl	8001d80 <SCSI_Address_Management>
 8001b4c:	4603      	mov	r3, r0
 8001b4e:	2b00      	cmp	r3, #0
 8001b50:	d02f      	beq.n	8001bb2 <SCSI_Write10_Cmd+0x8e>
    {
      return;
    }

    if ((CBW.bmFlags & 0x80) == 0)
 8001b52:	4b1b      	ldr	r3, [pc, #108]	; (8001bc0 <SCSI_Write10_Cmd+0x9c>)
 8001b54:	7b1b      	ldrb	r3, [r3, #12]
 8001b56:	b2db      	uxtb	r3, r3
 8001b58:	b25b      	sxtb	r3, r3
 8001b5a:	2b00      	cmp	r3, #0
 8001b5c:	db0a      	blt.n	8001b74 <SCSI_Write10_Cmd+0x50>
    {
      Bot_State = BOT_DATA_OUT;
 8001b5e:	4b17      	ldr	r3, [pc, #92]	; (8001bbc <SCSI_Write10_Cmd+0x98>)
 8001b60:	f04f 0201 	mov.w	r2, #1
 8001b64:	701a      	strb	r2, [r3, #0]
    #ifndef STM32F10X_CL
      SetEPRxStatus(ENDP2, EP_RX_VALID);
 8001b66:	f04f 0002 	mov.w	r0, #2
 8001b6a:	f44f 5140 	mov.w	r1, #12288	; 0x3000
 8001b6e:	f004 fa2b 	bl	8005fc8 <SetEPRxStatus>
    {
      Bot_Abort(DIR_IN);
      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
      Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
    }
    return;
 8001b72:	e01f      	b.n	8001bb4 <SCSI_Write10_Cmd+0x90>
      SetEPRxStatus(ENDP2, EP_RX_VALID);
    #endif /* STM32F10X_CL */
    }
    else
    {
      Bot_Abort(DIR_IN);
 8001b74:	f04f 0000 	mov.w	r0, #0
 8001b78:	f7ff fa8a 	bl	8001090 <Bot_Abort>
      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
 8001b7c:	4b10      	ldr	r3, [pc, #64]	; (8001bc0 <SCSI_Write10_Cmd+0x9c>)
 8001b7e:	7b5b      	ldrb	r3, [r3, #13]
 8001b80:	4618      	mov	r0, r3
 8001b82:	f04f 0105 	mov.w	r1, #5
 8001b86:	f04f 0224 	mov.w	r2, #36	; 0x24
 8001b8a:	f7ff ff55 	bl	8001a38 <Set_Scsi_Sense_Data>
      Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8001b8e:	f04f 0001 	mov.w	r0, #1
 8001b92:	f04f 0100 	mov.w	r1, #0
 8001b96:	f7ff fa4b 	bl	8001030 <Set_CSW>
    }
    return;
 8001b9a:	e00b      	b.n	8001bb4 <SCSI_Write10_Cmd+0x90>
  }
  else if (Bot_State == BOT_DATA_OUT)
 8001b9c:	4b07      	ldr	r3, [pc, #28]	; (8001bbc <SCSI_Write10_Cmd+0x98>)
 8001b9e:	781b      	ldrb	r3, [r3, #0]
 8001ba0:	2b01      	cmp	r3, #1
 8001ba2:	d107      	bne.n	8001bb4 <SCSI_Write10_Cmd+0x90>
  {
    Write_Memory(lun , LBA , BlockNbr);
 8001ba4:	7bfb      	ldrb	r3, [r7, #15]
 8001ba6:	4618      	mov	r0, r3
 8001ba8:	68b9      	ldr	r1, [r7, #8]
 8001baa:	687a      	ldr	r2, [r7, #4]
 8001bac:	f7fe fdd6 	bl	800075c <Write_Memory>
 8001bb0:	e000      	b.n	8001bb4 <SCSI_Write10_Cmd+0x90>

  if (Bot_State == BOT_IDLE)
  {
    if (!(SCSI_Address_Management(CBW.bLUN, SCSI_WRITE10 , LBA, BlockNbr)))/*address out of range*/
    {
      return;
 8001bb2:	bf00      	nop
  }
  else if (Bot_State == BOT_DATA_OUT)
  {
    Write_Memory(lun , LBA , BlockNbr);
  }
}
 8001bb4:	f107 0710 	add.w	r7, r7, #16
 8001bb8:	46bd      	mov	sp, r7
 8001bba:	bd80      	pop	{r7, pc}
 8001bbc:	2000047a 	.word	0x2000047a
 8001bc0:	20000458 	.word	0x20000458

08001bc4 <SCSI_Verify10_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Verify10_Cmd(uint8_t lun)
{
 8001bc4:	b580      	push	{r7, lr}
 8001bc6:	b082      	sub	sp, #8
 8001bc8:	af00      	add	r7, sp, #0
 8001bca:	4603      	mov	r3, r0
 8001bcc:	71fb      	strb	r3, [r7, #7]
  if ((CBW.dDataLength == 0) && !(CBW.CB[1] & BLKVFY))/* BLKVFY not set*/
 8001bce:	4b14      	ldr	r3, [pc, #80]	; (8001c20 <SCSI_Verify10_Cmd+0x5c>)
 8001bd0:	689b      	ldr	r3, [r3, #8]
 8001bd2:	2b00      	cmp	r3, #0
 8001bd4:	d10c      	bne.n	8001bf0 <SCSI_Verify10_Cmd+0x2c>
 8001bd6:	4b12      	ldr	r3, [pc, #72]	; (8001c20 <SCSI_Verify10_Cmd+0x5c>)
 8001bd8:	7c1b      	ldrb	r3, [r3, #16]
 8001bda:	f003 0304 	and.w	r3, r3, #4
 8001bde:	2b00      	cmp	r3, #0
 8001be0:	d106      	bne.n	8001bf0 <SCSI_Verify10_Cmd+0x2c>
  {
    Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
 8001be2:	f04f 0000 	mov.w	r0, #0
 8001be6:	f04f 0101 	mov.w	r1, #1
 8001bea:	f7ff fa21 	bl	8001030 <Set_CSW>
 8001bee:	e012      	b.n	8001c16 <SCSI_Verify10_Cmd+0x52>
  }
  else
  {
    Bot_Abort(BOTH_DIR);
 8001bf0:	f04f 0002 	mov.w	r0, #2
 8001bf4:	f7ff fa4c 	bl	8001090 <Bot_Abort>
    Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
 8001bf8:	4b09      	ldr	r3, [pc, #36]	; (8001c20 <SCSI_Verify10_Cmd+0x5c>)
 8001bfa:	7b5b      	ldrb	r3, [r3, #13]
 8001bfc:	4618      	mov	r0, r3
 8001bfe:	f04f 0105 	mov.w	r1, #5
 8001c02:	f04f 0224 	mov.w	r2, #36	; 0x24
 8001c06:	f7ff ff17 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8001c0a:	f04f 0001 	mov.w	r0, #1
 8001c0e:	f04f 0100 	mov.w	r1, #0
 8001c12:	f7ff fa0d 	bl	8001030 <Set_CSW>
  }
}
 8001c16:	f107 0708 	add.w	r7, r7, #8
 8001c1a:	46bd      	mov	sp, r7
 8001c1c:	bd80      	pop	{r7, pc}
 8001c1e:	bf00      	nop
 8001c20:	20000458 	.word	0x20000458

08001c24 <SCSI_Valid_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Valid_Cmd(uint8_t lun)
{
 8001c24:	b580      	push	{r7, lr}
 8001c26:	b082      	sub	sp, #8
 8001c28:	af00      	add	r7, sp, #0
 8001c2a:	4603      	mov	r3, r0
 8001c2c:	71fb      	strb	r3, [r7, #7]
  if (CBW.dDataLength != 0)
 8001c2e:	4b11      	ldr	r3, [pc, #68]	; (8001c74 <SCSI_Valid_Cmd+0x50>)
 8001c30:	689b      	ldr	r3, [r3, #8]
 8001c32:	2b00      	cmp	r3, #0
 8001c34:	d013      	beq.n	8001c5e <SCSI_Valid_Cmd+0x3a>
  {
    Bot_Abort(BOTH_DIR);
 8001c36:	f04f 0002 	mov.w	r0, #2
 8001c3a:	f7ff fa29 	bl	8001090 <Bot_Abort>
    Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_COMMAND);
 8001c3e:	4b0d      	ldr	r3, [pc, #52]	; (8001c74 <SCSI_Valid_Cmd+0x50>)
 8001c40:	7b5b      	ldrb	r3, [r3, #13]
 8001c42:	4618      	mov	r0, r3
 8001c44:	f04f 0105 	mov.w	r1, #5
 8001c48:	f04f 0220 	mov.w	r2, #32
 8001c4c:	f7ff fef4 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8001c50:	f04f 0001 	mov.w	r0, #1
 8001c54:	f04f 0100 	mov.w	r1, #0
 8001c58:	f7ff f9ea 	bl	8001030 <Set_CSW>
 8001c5c:	e005      	b.n	8001c6a <SCSI_Valid_Cmd+0x46>
  }
  else
    Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
 8001c5e:	f04f 0000 	mov.w	r0, #0
 8001c62:	f04f 0101 	mov.w	r1, #1
 8001c66:	f7ff f9e3 	bl	8001030 <Set_CSW>
}
 8001c6a:	f107 0708 	add.w	r7, r7, #8
 8001c6e:	46bd      	mov	sp, r7
 8001c70:	bd80      	pop	{r7, pc}
 8001c72:	bf00      	nop
 8001c74:	20000458 	.word	0x20000458

08001c78 <SCSI_TestUnitReady_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_TestUnitReady_Cmd(uint8_t lun)
{
 8001c78:	b580      	push	{r7, lr}
 8001c7a:	b082      	sub	sp, #8
 8001c7c:	af00      	add	r7, sp, #0
 8001c7e:	4603      	mov	r3, r0
 8001c80:	71fb      	strb	r3, [r7, #7]
  if (MAL_GetStatus(lun))
 8001c82:	79fb      	ldrb	r3, [r7, #7]
 8001c84:	4618      	mov	r0, r3
 8001c86:	f7fe fc4f 	bl	8000528 <MAL_GetStatus>
 8001c8a:	4603      	mov	r3, r0
 8001c8c:	2b00      	cmp	r3, #0
 8001c8e:	d013      	beq.n	8001cb8 <SCSI_TestUnitReady_Cmd+0x40>
  {
    Set_Scsi_Sense_Data(CBW.bLUN, NOT_READY, MEDIUM_NOT_PRESENT);
 8001c90:	4b0e      	ldr	r3, [pc, #56]	; (8001ccc <SCSI_TestUnitReady_Cmd+0x54>)
 8001c92:	7b5b      	ldrb	r3, [r3, #13]
 8001c94:	4618      	mov	r0, r3
 8001c96:	f04f 0102 	mov.w	r1, #2
 8001c9a:	f04f 023a 	mov.w	r2, #58	; 0x3a
 8001c9e:	f7ff fecb 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_ENABLE);
 8001ca2:	f04f 0001 	mov.w	r0, #1
 8001ca6:	f04f 0101 	mov.w	r1, #1
 8001caa:	f7ff f9c1 	bl	8001030 <Set_CSW>
    Bot_Abort(DIR_IN);
 8001cae:	f04f 0000 	mov.w	r0, #0
 8001cb2:	f7ff f9ed 	bl	8001090 <Bot_Abort>
    return;
 8001cb6:	e005      	b.n	8001cc4 <SCSI_TestUnitReady_Cmd+0x4c>
  }
  else
  {
    Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
 8001cb8:	f04f 0000 	mov.w	r0, #0
 8001cbc:	f04f 0101 	mov.w	r1, #1
 8001cc0:	f7ff f9b6 	bl	8001030 <Set_CSW>
  }
}
 8001cc4:	f107 0708 	add.w	r7, r7, #8
 8001cc8:	46bd      	mov	sp, r7
 8001cca:	bd80      	pop	{r7, pc}
 8001ccc:	20000458 	.word	0x20000458

08001cd0 <SCSI_Format_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Format_Cmd(uint8_t lun)
{
 8001cd0:	b580      	push	{r7, lr}
 8001cd2:	b082      	sub	sp, #8
 8001cd4:	af00      	add	r7, sp, #0
 8001cd6:	4603      	mov	r3, r0
 8001cd8:	71fb      	strb	r3, [r7, #7]
  if (MAL_GetStatus(lun))
 8001cda:	79fb      	ldrb	r3, [r7, #7]
 8001cdc:	4618      	mov	r0, r3
 8001cde:	f7fe fc23 	bl	8000528 <MAL_GetStatus>
 8001ce2:	4603      	mov	r3, r0
 8001ce4:	2b00      	cmp	r3, #0
 8001ce6:	d013      	beq.n	8001d10 <SCSI_Format_Cmd+0x40>
  {
    Set_Scsi_Sense_Data(CBW.bLUN, NOT_READY, MEDIUM_NOT_PRESENT);
 8001ce8:	4b0b      	ldr	r3, [pc, #44]	; (8001d18 <SCSI_Format_Cmd+0x48>)
 8001cea:	7b5b      	ldrb	r3, [r3, #13]
 8001cec:	4618      	mov	r0, r3
 8001cee:	f04f 0102 	mov.w	r1, #2
 8001cf2:	f04f 023a 	mov.w	r2, #58	; 0x3a
 8001cf6:	f7ff fe9f 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_ENABLE);
 8001cfa:	f04f 0001 	mov.w	r0, #1
 8001cfe:	f04f 0101 	mov.w	r1, #1
 8001d02:	f7ff f995 	bl	8001030 <Set_CSW>
    Bot_Abort(DIR_IN);
 8001d06:	f04f 0000 	mov.w	r0, #0
 8001d0a:	f7ff f9c1 	bl	8001090 <Bot_Abort>
    return;
 8001d0e:	bf00      	nop
  {
    NAND_Format();
    Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
  }
#endif
}
 8001d10:	f107 0708 	add.w	r7, r7, #8
 8001d14:	46bd      	mov	sp, r7
 8001d16:	bd80      	pop	{r7, pc}
 8001d18:	20000458 	.word	0x20000458

08001d1c <SCSI_Invalid_Cmd>:
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SCSI_Invalid_Cmd(uint8_t lun)
{
 8001d1c:	b580      	push	{r7, lr}
 8001d1e:	b082      	sub	sp, #8
 8001d20:	af00      	add	r7, sp, #0
 8001d22:	4603      	mov	r3, r0
 8001d24:	71fb      	strb	r3, [r7, #7]
  if (CBW.dDataLength == 0)
 8001d26:	4b15      	ldr	r3, [pc, #84]	; (8001d7c <SCSI_Invalid_Cmd+0x60>)
 8001d28:	689b      	ldr	r3, [r3, #8]
 8001d2a:	2b00      	cmp	r3, #0
 8001d2c:	d104      	bne.n	8001d38 <SCSI_Invalid_Cmd+0x1c>
  {
    Bot_Abort(DIR_IN);
 8001d2e:	f04f 0000 	mov.w	r0, #0
 8001d32:	f7ff f9ad 	bl	8001090 <Bot_Abort>
 8001d36:	e00e      	b.n	8001d56 <SCSI_Invalid_Cmd+0x3a>
  }
  else
  {
    if ((CBW.bmFlags & 0x80) != 0)
 8001d38:	4b10      	ldr	r3, [pc, #64]	; (8001d7c <SCSI_Invalid_Cmd+0x60>)
 8001d3a:	7b1b      	ldrb	r3, [r3, #12]
 8001d3c:	b2db      	uxtb	r3, r3
 8001d3e:	b25b      	sxtb	r3, r3
 8001d40:	2b00      	cmp	r3, #0
 8001d42:	da04      	bge.n	8001d4e <SCSI_Invalid_Cmd+0x32>
    {
      Bot_Abort(DIR_IN);
 8001d44:	f04f 0000 	mov.w	r0, #0
 8001d48:	f7ff f9a2 	bl	8001090 <Bot_Abort>
 8001d4c:	e003      	b.n	8001d56 <SCSI_Invalid_Cmd+0x3a>
    }
    else
    {
      Bot_Abort(BOTH_DIR);
 8001d4e:	f04f 0002 	mov.w	r0, #2
 8001d52:	f7ff f99d 	bl	8001090 <Bot_Abort>
    }
  }
  Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_COMMAND);
 8001d56:	4b09      	ldr	r3, [pc, #36]	; (8001d7c <SCSI_Invalid_Cmd+0x60>)
 8001d58:	7b5b      	ldrb	r3, [r3, #13]
 8001d5a:	4618      	mov	r0, r3
 8001d5c:	f04f 0105 	mov.w	r1, #5
 8001d60:	f04f 0220 	mov.w	r2, #32
 8001d64:	f7ff fe68 	bl	8001a38 <Set_Scsi_Sense_Data>
  Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8001d68:	f04f 0001 	mov.w	r0, #1
 8001d6c:	f04f 0100 	mov.w	r1, #0
 8001d70:	f7ff f95e 	bl	8001030 <Set_CSW>
}
 8001d74:	f107 0708 	add.w	r7, r7, #8
 8001d78:	46bd      	mov	sp, r7
 8001d7a:	bd80      	pop	{r7, pc}
 8001d7c:	20000458 	.word	0x20000458

08001d80 <SCSI_Address_Management>:
* Input          : uint8_t Cmd : the command can be SCSI_READ10 or SCSI_WRITE10.
* Output         : None.
* Return         : Read\Write status (bool).
*******************************************************************************/
bool SCSI_Address_Management(uint8_t lun , uint8_t Cmd , uint32_t LBA , uint32_t BlockNbr)
{
 8001d80:	b580      	push	{r7, lr}
 8001d82:	b084      	sub	sp, #16
 8001d84:	af00      	add	r7, sp, #0
 8001d86:	60ba      	str	r2, [r7, #8]
 8001d88:	607b      	str	r3, [r7, #4]
 8001d8a:	4603      	mov	r3, r0
 8001d8c:	73fb      	strb	r3, [r7, #15]
 8001d8e:	460b      	mov	r3, r1
 8001d90:	73bb      	strb	r3, [r7, #14]

  if ((LBA + BlockNbr) > Mass_Block_Count[lun] )
 8001d92:	68ba      	ldr	r2, [r7, #8]
 8001d94:	687b      	ldr	r3, [r7, #4]
 8001d96:	18d2      	adds	r2, r2, r3
 8001d98:	7bf9      	ldrb	r1, [r7, #15]
 8001d9a:	4b28      	ldr	r3, [pc, #160]	; (8001e3c <SCSI_Address_Management+0xbc>)
 8001d9c:	f853 3021 	ldr.w	r3, [r3, r1, lsl #2]
 8001da0:	429a      	cmp	r2, r3
 8001da2:	d91b      	bls.n	8001ddc <SCSI_Address_Management+0x5c>
  {
    if (Cmd == SCSI_WRITE10)
 8001da4:	7bbb      	ldrb	r3, [r7, #14]
 8001da6:	2b2a      	cmp	r3, #42	; 0x2a
 8001da8:	d103      	bne.n	8001db2 <SCSI_Address_Management+0x32>
    {
      Bot_Abort(BOTH_DIR);
 8001daa:	f04f 0002 	mov.w	r0, #2
 8001dae:	f7ff f96f 	bl	8001090 <Bot_Abort>
    }
    Bot_Abort(DIR_IN);
 8001db2:	f04f 0000 	mov.w	r0, #0
 8001db6:	f7ff f96b 	bl	8001090 <Bot_Abort>
    Set_Scsi_Sense_Data(lun, ILLEGAL_REQUEST, ADDRESS_OUT_OF_RANGE);
 8001dba:	7bfb      	ldrb	r3, [r7, #15]
 8001dbc:	4618      	mov	r0, r3
 8001dbe:	f04f 0105 	mov.w	r1, #5
 8001dc2:	f04f 0221 	mov.w	r2, #33	; 0x21
 8001dc6:	f7ff fe37 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8001dca:	f04f 0001 	mov.w	r0, #1
 8001dce:	f04f 0100 	mov.w	r1, #0
 8001dd2:	f7ff f92d 	bl	8001030 <Set_CSW>
    return (FALSE);
 8001dd6:	f04f 0300 	mov.w	r3, #0
 8001dda:	e02a      	b.n	8001e32 <SCSI_Address_Management+0xb2>
  }


  if (CBW.dDataLength != BlockNbr * Mass_Block_Size[lun])
 8001ddc:	4b18      	ldr	r3, [pc, #96]	; (8001e40 <SCSI_Address_Management+0xc0>)
 8001dde:	689a      	ldr	r2, [r3, #8]
 8001de0:	7bf9      	ldrb	r1, [r7, #15]
 8001de2:	4b18      	ldr	r3, [pc, #96]	; (8001e44 <SCSI_Address_Management+0xc4>)
 8001de4:	f853 3021 	ldr.w	r3, [r3, r1, lsl #2]
 8001de8:	6879      	ldr	r1, [r7, #4]
 8001dea:	fb01 f303 	mul.w	r3, r1, r3
 8001dee:	429a      	cmp	r2, r3
 8001df0:	d01d      	beq.n	8001e2e <SCSI_Address_Management+0xae>
  {
    if (Cmd == SCSI_WRITE10)
 8001df2:	7bbb      	ldrb	r3, [r7, #14]
 8001df4:	2b2a      	cmp	r3, #42	; 0x2a
 8001df6:	d104      	bne.n	8001e02 <SCSI_Address_Management+0x82>
    {
      Bot_Abort(BOTH_DIR);
 8001df8:	f04f 0002 	mov.w	r0, #2
 8001dfc:	f7ff f948 	bl	8001090 <Bot_Abort>
 8001e00:	e003      	b.n	8001e0a <SCSI_Address_Management+0x8a>
    }
    else
    {
      Bot_Abort(DIR_IN);
 8001e02:	f04f 0000 	mov.w	r0, #0
 8001e06:	f7ff f943 	bl	8001090 <Bot_Abort>
    }
    Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
 8001e0a:	4b0d      	ldr	r3, [pc, #52]	; (8001e40 <SCSI_Address_Management+0xc0>)
 8001e0c:	7b5b      	ldrb	r3, [r3, #13]
 8001e0e:	4618      	mov	r0, r3
 8001e10:	f04f 0105 	mov.w	r1, #5
 8001e14:	f04f 0224 	mov.w	r2, #36	; 0x24
 8001e18:	f7ff fe0e 	bl	8001a38 <Set_Scsi_Sense_Data>
    Set_CSW (CSW_CMD_FAILED, SEND_CSW_DISABLE);
 8001e1c:	f04f 0001 	mov.w	r0, #1
 8001e20:	f04f 0100 	mov.w	r1, #0
 8001e24:	f7ff f904 	bl	8001030 <Set_CSW>
    return (FALSE);
 8001e28:	f04f 0300 	mov.w	r3, #0
 8001e2c:	e001      	b.n	8001e32 <SCSI_Address_Management+0xb2>
  }
  return (TRUE);
 8001e2e:	f04f 0301 	mov.w	r3, #1
}
 8001e32:	4618      	mov	r0, r3
 8001e34:	f107 0710 	add.w	r7, r7, #16
 8001e38:	46bd      	mov	sp, r7
 8001e3a:	bd80      	pop	{r7, pc}
 8001e3c:	20000248 	.word	0x20000248
 8001e40:	20000458 	.word	0x20000458
 8001e44:	20000240 	.word	0x20000240

08001e48 <rprintfInit>:
static void (*rputchar)(unsigned char c);

// *** rprintf initialization ***
// you must call this function once and supply the character output
// routine before using other functions in this library
void rprintfInit(void (*putchar_func)(char c)) {
 8001e48:	b480      	push	{r7}
 8001e4a:	b083      	sub	sp, #12
 8001e4c:	af00      	add	r7, sp, #0
 8001e4e:	6078      	str	r0, [r7, #4]
	rputchar = putchar_func;
 8001e50:	4b03      	ldr	r3, [pc, #12]	; (8001e60 <rprintfInit+0x18>)
 8001e52:	687a      	ldr	r2, [r7, #4]
 8001e54:	601a      	str	r2, [r3, #0]
}
 8001e56:	f107 070c 	add.w	r7, r7, #12
 8001e5a:	46bd      	mov	sp, r7
 8001e5c:	bc80      	pop	{r7}
 8001e5e:	4770      	bx	lr
 8001e60:	20000230 	.word	0x20000230

08001e64 <rprintfChar>:

// *** rprintfChar ***
// send a character/byte to the current output device
void rprintfChar(unsigned char c) {
 8001e64:	b580      	push	{r7, lr}
 8001e66:	b082      	sub	sp, #8
 8001e68:	af00      	add	r7, sp, #0
 8001e6a:	4603      	mov	r3, r0
 8001e6c:	71fb      	strb	r3, [r7, #7]
	// do LF -> CR/LF translation
 	if(c == '\n')
 8001e6e:	79fb      	ldrb	r3, [r7, #7]
 8001e70:	2b0a      	cmp	r3, #10
 8001e72:	d104      	bne.n	8001e7e <rprintfChar+0x1a>
		rputchar('\r');
 8001e74:	4b06      	ldr	r3, [pc, #24]	; (8001e90 <rprintfChar+0x2c>)
 8001e76:	681b      	ldr	r3, [r3, #0]
 8001e78:	f04f 000d 	mov.w	r0, #13
 8001e7c:	4798      	blx	r3
	rputchar(c);	// send character
 8001e7e:	4b04      	ldr	r3, [pc, #16]	; (8001e90 <rprintfChar+0x2c>)
 8001e80:	681b      	ldr	r3, [r3, #0]
 8001e82:	79fa      	ldrb	r2, [r7, #7]
 8001e84:	4610      	mov	r0, r2
 8001e86:	4798      	blx	r3
}
 8001e88:	f107 0708 	add.w	r7, r7, #8
 8001e8c:	46bd      	mov	sp, r7
 8001e8e:	bd80      	pop	{r7, pc}
 8001e90:	20000230 	.word	0x20000230

08001e94 <rprintfStr>:

// *** rprintfStr ***
// prints a null-terminated string stored in RAM
void rprintfStr(char str[]) {	// send a string stored in RAM
 8001e94:	b580      	push	{r7, lr}
 8001e96:	b082      	sub	sp, #8
 8001e98:	af00      	add	r7, sp, #0
 8001e9a:	6078      	str	r0, [r7, #4]
	// check to make sure we have a good pointer
	if (!str) return;
 8001e9c:	687b      	ldr	r3, [r7, #4]
 8001e9e:	2b00      	cmp	r3, #0
 8001ea0:	d10a      	bne.n	8001eb8 <rprintfStr+0x24>
 8001ea2:	e00e      	b.n	8001ec2 <rprintfStr+0x2e>
	// print the string until a null-terminator
	while (*str)
		rprintfChar(*str++);
 8001ea4:	687b      	ldr	r3, [r7, #4]
 8001ea6:	781b      	ldrb	r3, [r3, #0]
 8001ea8:	687a      	ldr	r2, [r7, #4]
 8001eaa:	f102 0201 	add.w	r2, r2, #1
 8001eae:	607a      	str	r2, [r7, #4]
 8001eb0:	4618      	mov	r0, r3
 8001eb2:	f7ff ffd7 	bl	8001e64 <rprintfChar>
 8001eb6:	e000      	b.n	8001eba <rprintfStr+0x26>
// prints a null-terminated string stored in RAM
void rprintfStr(char str[]) {	// send a string stored in RAM
	// check to make sure we have a good pointer
	if (!str) return;
	// print the string until a null-terminator
	while (*str)
 8001eb8:	bf00      	nop
 8001eba:	687b      	ldr	r3, [r7, #4]
 8001ebc:	781b      	ldrb	r3, [r3, #0]
 8001ebe:	2b00      	cmp	r3, #0
 8001ec0:	d1f0      	bne.n	8001ea4 <rprintfStr+0x10>
		rprintfChar(*str++);
}
 8001ec2:	f107 0708 	add.w	r7, r7, #8
 8001ec6:	46bd      	mov	sp, r7
 8001ec8:	bd80      	pop	{r7, pc}
 8001eca:	bf00      	nop

08001ecc <rprintfStrLen>:

// *** rprintfStrLen ***
// prints a section of a string stored in RAM
// begins printing at position indicated by <start>
// prints number of characters indicated by <len>
void rprintfStrLen(char str[], unsigned int start, unsigned int len) {
 8001ecc:	b580      	push	{r7, lr}
 8001ece:	b086      	sub	sp, #24
 8001ed0:	af00      	add	r7, sp, #0
 8001ed2:	60f8      	str	r0, [r7, #12]
 8001ed4:	60b9      	str	r1, [r7, #8]
 8001ed6:	607a      	str	r2, [r7, #4]
	unsigned int i=0;
 8001ed8:	f04f 0300 	mov.w	r3, #0
 8001edc:	617b      	str	r3, [r7, #20]
	// check to make sure we have a good pointer
	if (!str) return;
 8001ede:	68fb      	ldr	r3, [r7, #12]
 8001ee0:	2b00      	cmp	r3, #0
 8001ee2:	d039      	beq.n	8001f58 <rprintfStrLen+0x8c>
	// spin through characters up to requested start
	// keep going as long as there's no null
	while((i++<start) && (*str++));
 8001ee4:	bf00      	nop
 8001ee6:	697a      	ldr	r2, [r7, #20]
 8001ee8:	68bb      	ldr	r3, [r7, #8]
 8001eea:	429a      	cmp	r2, r3
 8001eec:	bf2c      	ite	cs
 8001eee:	2300      	movcs	r3, #0
 8001ef0:	2301      	movcc	r3, #1
 8001ef2:	b2db      	uxtb	r3, r3
 8001ef4:	697a      	ldr	r2, [r7, #20]
 8001ef6:	f102 0201 	add.w	r2, r2, #1
 8001efa:	617a      	str	r2, [r7, #20]
 8001efc:	2b00      	cmp	r3, #0
 8001efe:	d00c      	beq.n	8001f1a <rprintfStrLen+0x4e>
 8001f00:	68fb      	ldr	r3, [r7, #12]
 8001f02:	781b      	ldrb	r3, [r3, #0]
 8001f04:	2b00      	cmp	r3, #0
 8001f06:	bf0c      	ite	eq
 8001f08:	2300      	moveq	r3, #0
 8001f0a:	2301      	movne	r3, #1
 8001f0c:	b2db      	uxtb	r3, r3
 8001f0e:	68fa      	ldr	r2, [r7, #12]
 8001f10:	f102 0201 	add.w	r2, r2, #1
 8001f14:	60fa      	str	r2, [r7, #12]
 8001f16:	2b00      	cmp	r3, #0
 8001f18:	d1e5      	bne.n	8001ee6 <rprintfStrLen+0x1a>
//	{
//		// keep steping through string as long as there's no null
//		if(*str) str++;
//	}
	// then print exactly len characters
	for(i=0; i<len; i++)
 8001f1a:	f04f 0300 	mov.w	r3, #0
 8001f1e:	617b      	str	r3, [r7, #20]
 8001f20:	e015      	b.n	8001f4e <rprintfStrLen+0x82>
	{
		// print data out of the string as long as we haven't reached a null yet
		// at the null, start printing spaces
		if(*str)
 8001f22:	68fb      	ldr	r3, [r7, #12]
 8001f24:	781b      	ldrb	r3, [r3, #0]
 8001f26:	2b00      	cmp	r3, #0
 8001f28:	d009      	beq.n	8001f3e <rprintfStrLen+0x72>
			rprintfChar(*str++);
 8001f2a:	68fb      	ldr	r3, [r7, #12]
 8001f2c:	781b      	ldrb	r3, [r3, #0]
 8001f2e:	68fa      	ldr	r2, [r7, #12]
 8001f30:	f102 0201 	add.w	r2, r2, #1
 8001f34:	60fa      	str	r2, [r7, #12]
 8001f36:	4618      	mov	r0, r3
 8001f38:	f7ff ff94 	bl	8001e64 <rprintfChar>
 8001f3c:	e003      	b.n	8001f46 <rprintfStrLen+0x7a>
		else
			rprintfChar(' ');
 8001f3e:	f04f 0020 	mov.w	r0, #32
 8001f42:	f7ff ff8f 	bl	8001e64 <rprintfChar>
//	{
//		// keep steping through string as long as there's no null
//		if(*str) str++;
//	}
	// then print exactly len characters
	for(i=0; i<len; i++)
 8001f46:	697b      	ldr	r3, [r7, #20]
 8001f48:	f103 0301 	add.w	r3, r3, #1
 8001f4c:	617b      	str	r3, [r7, #20]
 8001f4e:	697a      	ldr	r2, [r7, #20]
 8001f50:	687b      	ldr	r3, [r7, #4]
 8001f52:	429a      	cmp	r2, r3
 8001f54:	d3e5      	bcc.n	8001f22 <rprintfStrLen+0x56>
 8001f56:	e000      	b.n	8001f5a <rprintfStrLen+0x8e>
// begins printing at position indicated by <start>
// prints number of characters indicated by <len>
void rprintfStrLen(char str[], unsigned int start, unsigned int len) {
	unsigned int i=0;
	// check to make sure we have a good pointer
	if (!str) return;
 8001f58:	bf00      	nop
			rprintfChar(*str++);
		else
			rprintfChar(' ');
	}

}
 8001f5a:	f107 0718 	add.w	r7, r7, #24
 8001f5e:	46bd      	mov	sp, r7
 8001f60:	bd80      	pop	{r7, pc}
 8001f62:	bf00      	nop

08001f64 <rprintfCRLF>:

// *** rprintfCRLF ***
// prints carriage return and line feed
void rprintfCRLF(void) {
 8001f64:	b580      	push	{r7, lr}
 8001f66:	af00      	add	r7, sp, #0
	// print CR/LF
	//rprintfChar('\r');
	// LF -> CR/LF translation built-in to rprintfChar()
	rprintfChar('\n');
 8001f68:	f04f 000a 	mov.w	r0, #10
 8001f6c:	f7ff ff7a 	bl	8001e64 <rprintfChar>
}
 8001f70:	bd80      	pop	{r7, pc}
 8001f72:	bf00      	nop

08001f74 <rprintfu04>:

// *** rprintfu04 ***
// prints an unsigned 4-bit number in hex (1 digit)
void rprintfu04(unsigned char data) {
 8001f74:	b580      	push	{r7, lr}
 8001f76:	b082      	sub	sp, #8
 8001f78:	af00      	add	r7, sp, #0
 8001f7a:	4603      	mov	r3, r0
 8001f7c:	71fb      	strb	r3, [r7, #7]
//	char Character = data&0x0f;
//	if (Character>9)
//		Character+='A'-10;
//	else
//		Character+='0';
	rprintfChar(HexChars[data]);
 8001f7e:	79fb      	ldrb	r3, [r7, #7]
 8001f80:	4a04      	ldr	r2, [pc, #16]	; (8001f94 <rprintfu04+0x20>)
 8001f82:	5cd3      	ldrb	r3, [r2, r3]
 8001f84:	4618      	mov	r0, r3
 8001f86:	f7ff ff6d 	bl	8001e64 <rprintfChar>
}
 8001f8a:	f107 0708 	add.w	r7, r7, #8
 8001f8e:	46bd      	mov	sp, r7
 8001f90:	bd80      	pop	{r7, pc}
 8001f92:	bf00      	nop
 8001f94:	08007748 	.word	0x08007748

08001f98 <rprintfu08>:

// *** rprintfu08 ***
// prints an unsigned 8-bit number in hex (2 digits)
void rprintfu08(unsigned char data) {
 8001f98:	b580      	push	{r7, lr}
 8001f9a:	b082      	sub	sp, #8
 8001f9c:	af00      	add	r7, sp, #0
 8001f9e:	4603      	mov	r3, r0
 8001fa0:	71fb      	strb	r3, [r7, #7]
	// print 8-bit hex value
	rprintfu04(data>>4);
 8001fa2:	79fb      	ldrb	r3, [r7, #7]
 8001fa4:	ea4f 1313 	mov.w	r3, r3, lsr #4
 8001fa8:	b2db      	uxtb	r3, r3
 8001faa:	4618      	mov	r0, r3
 8001fac:	f7ff ffe2 	bl	8001f74 <rprintfu04>
	rprintfu04(data);
 8001fb0:	79fb      	ldrb	r3, [r7, #7]
 8001fb2:	4618      	mov	r0, r3
 8001fb4:	f7ff ffde 	bl	8001f74 <rprintfu04>
}
 8001fb8:	f107 0708 	add.w	r7, r7, #8
 8001fbc:	46bd      	mov	sp, r7
 8001fbe:	bd80      	pop	{r7, pc}

08001fc0 <rprintfu16>:

// *** rprintfu16 ***
// prints an unsigned 16-bit number in hex (4 digits)
void rprintfu16(unsigned short data) {
 8001fc0:	b580      	push	{r7, lr}
 8001fc2:	b082      	sub	sp, #8
 8001fc4:	af00      	add	r7, sp, #0
 8001fc6:	4603      	mov	r3, r0
 8001fc8:	80fb      	strh	r3, [r7, #6]
	// print 16-bit hex value
	rprintfu08(data>>8);
 8001fca:	88fb      	ldrh	r3, [r7, #6]
 8001fcc:	ea4f 2313 	mov.w	r3, r3, lsr #8
 8001fd0:	b29b      	uxth	r3, r3
 8001fd2:	b2db      	uxtb	r3, r3
 8001fd4:	4618      	mov	r0, r3
 8001fd6:	f7ff ffdf 	bl	8001f98 <rprintfu08>
	rprintfu08(data);
 8001fda:	88fb      	ldrh	r3, [r7, #6]
 8001fdc:	b2db      	uxtb	r3, r3
 8001fde:	4618      	mov	r0, r3
 8001fe0:	f7ff ffda 	bl	8001f98 <rprintfu08>
}
 8001fe4:	f107 0708 	add.w	r7, r7, #8
 8001fe8:	46bd      	mov	sp, r7
 8001fea:	bd80      	pop	{r7, pc}

08001fec <rprintfu32>:

// *** rprintfu32 ***
// prints an unsigned 32-bit number in hex (8 digits)
void rprintfu32(unsigned long data) {
 8001fec:	b580      	push	{r7, lr}
 8001fee:	b082      	sub	sp, #8
 8001ff0:	af00      	add	r7, sp, #0
 8001ff2:	6078      	str	r0, [r7, #4]
	// print 32-bit hex value
	rprintfu16(data>>16);
 8001ff4:	687b      	ldr	r3, [r7, #4]
 8001ff6:	ea4f 4313 	mov.w	r3, r3, lsr #16
 8001ffa:	b29b      	uxth	r3, r3
 8001ffc:	4618      	mov	r0, r3
 8001ffe:	f7ff ffdf 	bl	8001fc0 <rprintfu16>
	rprintfu16(data);
 8002002:	687b      	ldr	r3, [r7, #4]
 8002004:	b29b      	uxth	r3, r3
 8002006:	4618      	mov	r0, r3
 8002008:	f7ff ffda 	bl	8001fc0 <rprintfu16>
}
 800200c:	f107 0708 	add.w	r7, r7, #8
 8002010:	46bd      	mov	sp, r7
 8002012:	bd80      	pop	{r7, pc}

08002014 <rprintfNum>:
//
//	Examples:
//	uartPrintfNum(10, 6,  TRUE, ' ',   1234);  -->  " +1234"
//	uartPrintfNum(10, 6, FALSE, '0',   1234);  -->  "001234"
//	uartPrintfNum(16, 6, FALSE, '.', 0x5AA5);  -->  "..5AA5"
void rprintfNum(char base, char numDigits, char isSigned, char padchar, long n) {
 8002014:	b580      	push	{r7, lr}
 8002016:	b08e      	sub	sp, #56	; 0x38
 8002018:	af00      	add	r7, sp, #0
 800201a:	71f8      	strb	r0, [r7, #7]
 800201c:	71b9      	strb	r1, [r7, #6]
 800201e:	717a      	strb	r2, [r7, #5]
 8002020:	713b      	strb	r3, [r7, #4]
	char *p, buf_[32];
	unsigned long x;
	unsigned char count;

	// prepare negative number
	if( isSigned && (n < 0) )
 8002022:	797b      	ldrb	r3, [r7, #5]
 8002024:	2b00      	cmp	r3, #0
 8002026:	d007      	beq.n	8002038 <rprintfNum+0x24>
 8002028:	6c3b      	ldr	r3, [r7, #64]	; 0x40
 800202a:	2b00      	cmp	r3, #0
 800202c:	da04      	bge.n	8002038 <rprintfNum+0x24>
		x = -n;
 800202e:	6c3b      	ldr	r3, [r7, #64]	; 0x40
 8002030:	f1c3 0300 	rsb	r3, r3, #0
 8002034:	633b      	str	r3, [r7, #48]	; 0x30
 8002036:	e001      	b.n	800203c <rprintfNum+0x28>
	else
	 	x = n;
 8002038:	6c3b      	ldr	r3, [r7, #64]	; 0x40
 800203a:	633b      	str	r3, [r7, #48]	; 0x30
	// setup little string buffer
	count = (numDigits-1)-(isSigned?1:0);
 800203c:	797b      	ldrb	r3, [r7, #5]
 800203e:	2b00      	cmp	r3, #0
 8002040:	bf0c      	ite	eq
 8002042:	2300      	moveq	r3, #0
 8002044:	2301      	movne	r3, #1
 8002046:	b2db      	uxtb	r3, r3
 8002048:	79ba      	ldrb	r2, [r7, #6]
 800204a:	1ad3      	subs	r3, r2, r3
 800204c:	b2db      	uxtb	r3, r3
 800204e:	f103 33ff 	add.w	r3, r3, #4294967295
 8002052:	f887 302f 	strb.w	r3, [r7, #47]	; 0x2f
  	p = buf_ + sizeof (buf_);
 8002056:	f107 030c 	add.w	r3, r7, #12
 800205a:	f103 0320 	add.w	r3, r3, #32
 800205e:	637b      	str	r3, [r7, #52]	; 0x34
  	*--p = '\0';
 8002060:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002062:	f103 33ff 	add.w	r3, r3, #4294967295
 8002066:	637b      	str	r3, [r7, #52]	; 0x34
 8002068:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800206a:	f04f 0200 	mov.w	r2, #0
 800206e:	701a      	strb	r2, [r3, #0]
	// force calculation of first digit
	// (to prevent zero from not printing at all!!!)
	*--p = HexChars[x%base]; x /= base;
 8002070:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002072:	f103 33ff 	add.w	r3, r3, #4294967295
 8002076:	637b      	str	r3, [r7, #52]	; 0x34
 8002078:	79fa      	ldrb	r2, [r7, #7]
 800207a:	6b3b      	ldr	r3, [r7, #48]	; 0x30
 800207c:	fbb3 f1f2 	udiv	r1, r3, r2
 8002080:	fb02 f201 	mul.w	r2, r2, r1
 8002084:	1a9b      	subs	r3, r3, r2
 8002086:	4a3d      	ldr	r2, [pc, #244]	; (800217c <rprintfNum+0x168>)
 8002088:	5cd2      	ldrb	r2, [r2, r3]
 800208a:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800208c:	701a      	strb	r2, [r3, #0]
 800208e:	79fb      	ldrb	r3, [r7, #7]
 8002090:	6b3a      	ldr	r2, [r7, #48]	; 0x30
 8002092:	fbb2 f3f3 	udiv	r3, r2, r3
 8002096:	633b      	str	r3, [r7, #48]	; 0x30
	// calculate remaining digits
	while(count--) {
 8002098:	e01e      	b.n	80020d8 <rprintfNum+0xc4>
		if(x != 0){
 800209a:	6b3b      	ldr	r3, [r7, #48]	; 0x30
 800209c:	2b00      	cmp	r3, #0
 800209e:	d014      	beq.n	80020ca <rprintfNum+0xb6>
			*--p = HexChars[x%base]; x /= base;}// calculate next digit
 80020a0:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80020a2:	f103 33ff 	add.w	r3, r3, #4294967295
 80020a6:	637b      	str	r3, [r7, #52]	; 0x34
 80020a8:	79fa      	ldrb	r2, [r7, #7]
 80020aa:	6b3b      	ldr	r3, [r7, #48]	; 0x30
 80020ac:	fbb3 f1f2 	udiv	r1, r3, r2
 80020b0:	fb02 f201 	mul.w	r2, r2, r1
 80020b4:	1a9b      	subs	r3, r3, r2
 80020b6:	4a31      	ldr	r2, [pc, #196]	; (800217c <rprintfNum+0x168>)
 80020b8:	5cd2      	ldrb	r2, [r2, r3]
 80020ba:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80020bc:	701a      	strb	r2, [r3, #0]
 80020be:	79fb      	ldrb	r3, [r7, #7]
 80020c0:	6b3a      	ldr	r2, [r7, #48]	; 0x30
 80020c2:	fbb2 f3f3 	udiv	r3, r2, r3
 80020c6:	633b      	str	r3, [r7, #48]	; 0x30
 80020c8:	e006      	b.n	80020d8 <rprintfNum+0xc4>
		else
			*--p = padchar;// no more digits left, pad out to desired length
 80020ca:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80020cc:	f103 33ff 	add.w	r3, r3, #4294967295
 80020d0:	637b      	str	r3, [r7, #52]	; 0x34
 80020d2:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80020d4:	793a      	ldrb	r2, [r7, #4]
 80020d6:	701a      	strb	r2, [r3, #0]
  	*--p = '\0';
	// force calculation of first digit
	// (to prevent zero from not printing at all!!!)
	*--p = HexChars[x%base]; x /= base;
	// calculate remaining digits
	while(count--) {
 80020d8:	f897 302f 	ldrb.w	r3, [r7, #47]	; 0x2f
 80020dc:	2b00      	cmp	r3, #0
 80020de:	bf0c      	ite	eq
 80020e0:	2300      	moveq	r3, #0
 80020e2:	2301      	movne	r3, #1
 80020e4:	b2db      	uxtb	r3, r3
 80020e6:	f897 202f 	ldrb.w	r2, [r7, #47]	; 0x2f
 80020ea:	f102 32ff 	add.w	r2, r2, #4294967295
 80020ee:	f887 202f 	strb.w	r2, [r7, #47]	; 0x2f
 80020f2:	2b00      	cmp	r3, #0
 80020f4:	d1d1      	bne.n	800209a <rprintfNum+0x86>
		if(x != 0){
			*--p = HexChars[x%base]; x /= base;}// calculate next digit
		else
			*--p = padchar;// no more digits left, pad out to desired length
	}// apply signed notation if requested
	if( isSigned ) {
 80020f6:	797b      	ldrb	r3, [r7, #5]
 80020f8:	2b00      	cmp	r3, #0
 80020fa:	d01f      	beq.n	800213c <rprintfNum+0x128>
		if(n < 0)
 80020fc:	6c3b      	ldr	r3, [r7, #64]	; 0x40
 80020fe:	2b00      	cmp	r3, #0
 8002100:	da08      	bge.n	8002114 <rprintfNum+0x100>
   			*--p = '-';
 8002102:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002104:	f103 33ff 	add.w	r3, r3, #4294967295
 8002108:	637b      	str	r3, [r7, #52]	; 0x34
 800210a:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800210c:	f04f 022d 	mov.w	r2, #45	; 0x2d
 8002110:	701a      	strb	r2, [r3, #0]
 8002112:	e013      	b.n	800213c <rprintfNum+0x128>
		else if(n > 0)
 8002114:	6c3b      	ldr	r3, [r7, #64]	; 0x40
 8002116:	2b00      	cmp	r3, #0
 8002118:	dd08      	ble.n	800212c <rprintfNum+0x118>
	   		*--p = '+';
 800211a:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800211c:	f103 33ff 	add.w	r3, r3, #4294967295
 8002120:	637b      	str	r3, [r7, #52]	; 0x34
 8002122:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002124:	f04f 022b 	mov.w	r2, #43	; 0x2b
 8002128:	701a      	strb	r2, [r3, #0]
 800212a:	e007      	b.n	800213c <rprintfNum+0x128>
		else
	   		*--p = ' ';
 800212c:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800212e:	f103 33ff 	add.w	r3, r3, #4294967295
 8002132:	637b      	str	r3, [r7, #52]	; 0x34
 8002134:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002136:	f04f 0220 	mov.w	r2, #32
 800213a:	701a      	strb	r2, [r3, #0]
	}// print the string right-justified
	count = numDigits;
 800213c:	79bb      	ldrb	r3, [r7, #6]
 800213e:	f887 302f 	strb.w	r3, [r7, #47]	; 0x2f
	while(count--)
 8002142:	e008      	b.n	8002156 <rprintfNum+0x142>
		rprintfChar(*p++);
 8002144:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002146:	781b      	ldrb	r3, [r3, #0]
 8002148:	6b7a      	ldr	r2, [r7, #52]	; 0x34
 800214a:	f102 0201 	add.w	r2, r2, #1
 800214e:	637a      	str	r2, [r7, #52]	; 0x34
 8002150:	4618      	mov	r0, r3
 8002152:	f7ff fe87 	bl	8001e64 <rprintfChar>
	   		*--p = '+';
		else
	   		*--p = ' ';
	}// print the string right-justified
	count = numDigits;
	while(count--)
 8002156:	f897 302f 	ldrb.w	r3, [r7, #47]	; 0x2f
 800215a:	2b00      	cmp	r3, #0
 800215c:	bf0c      	ite	eq
 800215e:	2300      	moveq	r3, #0
 8002160:	2301      	movne	r3, #1
 8002162:	b2db      	uxtb	r3, r3
 8002164:	f897 202f 	ldrb.w	r2, [r7, #47]	; 0x2f
 8002168:	f102 32ff 	add.w	r2, r2, #4294967295
 800216c:	f887 202f 	strb.w	r2, [r7, #47]	; 0x2f
 8002170:	2b00      	cmp	r3, #0
 8002172:	d1e7      	bne.n	8002144 <rprintfNum+0x130>
		rprintfChar(*p++);
}
 8002174:	f107 0738 	add.w	r7, r7, #56	; 0x38
 8002178:	46bd      	mov	sp, r7
 800217a:	bd80      	pop	{r7, pc}
 800217c:	08007748 	.word	0x08007748

08002180 <rprintfFloat>:

#ifdef RPRINTF_FLOAT
// *** rprintfFloat ***
// floating-point print
void rprintfFloat(char numDigits, float x) {
 8002180:	b590      	push	{r4, r7, lr}
 8002182:	b087      	sub	sp, #28
 8002184:	af00      	add	r7, sp, #0
 8002186:	4603      	mov	r3, r0
 8002188:	6039      	str	r1, [r7, #0]
 800218a:	71fb      	strb	r3, [r7, #7]
        unsigned char sig = TRUE;
 800218c:	f04f 0301 	mov.w	r3, #1
 8002190:	75fb      	strb	r3, [r7, #23]
        unsigned char i, digit;
        float place = 1.0;
 8002192:	4b51      	ldr	r3, [pc, #324]	; (80022d8 <rprintfFloat+0x158>)
 8002194:	613b      	str	r3, [r7, #16]
        // print polarity character
        if(x<0)
 8002196:	f04f 0301 	mov.w	r3, #1
 800219a:	461c      	mov	r4, r3
 800219c:	6838      	ldr	r0, [r7, #0]
 800219e:	494f      	ldr	r1, [pc, #316]	; (80022dc <rprintfFloat+0x15c>)
 80021a0:	f005 f9cc 	bl	800753c <__aeabi_fcmplt>
 80021a4:	4603      	mov	r3, r0
 80021a6:	2b00      	cmp	r3, #0
 80021a8:	d102      	bne.n	80021b0 <rprintfFloat+0x30>
 80021aa:	f04f 0300 	mov.w	r3, #0
 80021ae:	461c      	mov	r4, r3
 80021b0:	b2e3      	uxtb	r3, r4
 80021b2:	2b00      	cmp	r3, #0
 80021b4:	d003      	beq.n	80021be <rprintfFloat+0x3e>
                rprintfChar('-');
 80021b6:	f04f 002d 	mov.w	r0, #45	; 0x2d
 80021ba:	f7ff fe53 	bl	8001e64 <rprintfChar>
        // convert to absolute value
        x = (x>0)?(x):(-x);
 80021be:	f04f 0301 	mov.w	r3, #1
 80021c2:	461c      	mov	r4, r3
 80021c4:	6838      	ldr	r0, [r7, #0]
 80021c6:	4945      	ldr	r1, [pc, #276]	; (80022dc <rprintfFloat+0x15c>)
 80021c8:	f005 f9d6 	bl	8007578 <__aeabi_fcmpgt>
 80021cc:	4603      	mov	r3, r0
 80021ce:	2b00      	cmp	r3, #0
 80021d0:	d102      	bne.n	80021d8 <rprintfFloat+0x58>
 80021d2:	f04f 0300 	mov.w	r3, #0
 80021d6:	461c      	mov	r4, r3
 80021d8:	b2e3      	uxtb	r3, r4
 80021da:	2b00      	cmp	r3, #0
 80021dc:	d001      	beq.n	80021e2 <rprintfFloat+0x62>
 80021de:	683b      	ldr	r3, [r7, #0]
 80021e0:	e002      	b.n	80021e8 <rprintfFloat+0x68>
 80021e2:	683b      	ldr	r3, [r7, #0]
 80021e4:	f083 4300 	eor.w	r3, r3, #2147483648	; 0x80000000
 80021e8:	603b      	str	r3, [r7, #0]
        // find starting digit place
        for(i=0; i<15; i++) {
 80021ea:	f04f 0300 	mov.w	r3, #0
 80021ee:	75bb      	strb	r3, [r7, #22]
 80021f0:	e01e      	b.n	8002230 <rprintfFloat+0xb0>
                if((x/place) < 10.0)
 80021f2:	6838      	ldr	r0, [r7, #0]
 80021f4:	6939      	ldr	r1, [r7, #16]
 80021f6:	f005 f8b7 	bl	8007368 <__aeabi_fdiv>
 80021fa:	4603      	mov	r3, r0
 80021fc:	f04f 0201 	mov.w	r2, #1
 8002200:	4614      	mov	r4, r2
 8002202:	4618      	mov	r0, r3
 8002204:	4936      	ldr	r1, [pc, #216]	; (80022e0 <rprintfFloat+0x160>)
 8002206:	f005 f999 	bl	800753c <__aeabi_fcmplt>
 800220a:	4603      	mov	r3, r0
 800220c:	2b00      	cmp	r3, #0
 800220e:	d102      	bne.n	8002216 <rprintfFloat+0x96>
 8002210:	f04f 0300 	mov.w	r3, #0
 8002214:	461c      	mov	r4, r3
 8002216:	b2e3      	uxtb	r3, r4
 8002218:	2b00      	cmp	r3, #0
 800221a:	d10d      	bne.n	8002238 <rprintfFloat+0xb8>
                        break;
                else
                        place *= 10.0;
 800221c:	6938      	ldr	r0, [r7, #16]
 800221e:	4930      	ldr	r1, [pc, #192]	; (80022e0 <rprintfFloat+0x160>)
 8002220:	f004 ffee 	bl	8007200 <__aeabi_fmul>
 8002224:	4603      	mov	r3, r0
 8002226:	613b      	str	r3, [r7, #16]
        if(x<0)
                rprintfChar('-');
        // convert to absolute value
        x = (x>0)?(x):(-x);
        // find starting digit place
        for(i=0; i<15; i++) {
 8002228:	7dbb      	ldrb	r3, [r7, #22]
 800222a:	f103 0301 	add.w	r3, r3, #1
 800222e:	75bb      	strb	r3, [r7, #22]
 8002230:	7dbb      	ldrb	r3, [r7, #22]
 8002232:	2b0e      	cmp	r3, #14
 8002234:	d9dd      	bls.n	80021f2 <rprintfFloat+0x72>
 8002236:	e000      	b.n	800223a <rprintfFloat+0xba>
                if((x/place) < 10.0)
                        break;
 8002238:	bf00      	nop
                else
                        place *= 10.0;
        }
        // print digits
        for(i=0; i<numDigits || sig; i++) {
 800223a:	f04f 0300 	mov.w	r3, #0
 800223e:	75bb      	strb	r3, [r7, #22]
 8002240:	e03e      	b.n	80022c0 <rprintfFloat+0x140>
                digit = (unsigned char)(x/place);
 8002242:	6838      	ldr	r0, [r7, #0]
 8002244:	6939      	ldr	r1, [r7, #16]
 8002246:	f005 f88f 	bl	8007368 <__aeabi_fdiv>
 800224a:	4603      	mov	r3, r0
 800224c:	4618      	mov	r0, r3
 800224e:	f005 f99d 	bl	800758c <__aeabi_f2uiz>
 8002252:	4603      	mov	r3, r0
 8002254:	73fb      	strb	r3, [r7, #15]
                rprintfChar(digit+0x30);
 8002256:	7bfb      	ldrb	r3, [r7, #15]
 8002258:	f103 0330 	add.w	r3, r3, #48	; 0x30
 800225c:	b2db      	uxtb	r3, r3
 800225e:	4618      	mov	r0, r3
 8002260:	f7ff fe00 	bl	8001e64 <rprintfChar>
                if(place == 1.0){
 8002264:	6938      	ldr	r0, [r7, #16]
 8002266:	491c      	ldr	r1, [pc, #112]	; (80022d8 <rprintfFloat+0x158>)
 8002268:	f005 f95e 	bl	8007528 <__aeabi_fcmpeq>
 800226c:	4603      	mov	r3, r0
 800226e:	2b00      	cmp	r3, #0
 8002270:	d00c      	beq.n	800228c <rprintfFloat+0x10c>
                        if((i+1)<numDigits)rprintfChar('.');
 8002272:	7dbb      	ldrb	r3, [r7, #22]
 8002274:	f103 0201 	add.w	r2, r3, #1
 8002278:	79fb      	ldrb	r3, [r7, #7]
 800227a:	429a      	cmp	r2, r3
 800227c:	da03      	bge.n	8002286 <rprintfFloat+0x106>
 800227e:	f04f 002e 	mov.w	r0, #46	; 0x2e
 8002282:	f7ff fdef 	bl	8001e64 <rprintfChar>
                        sig=FALSE;
 8002286:	f04f 0300 	mov.w	r3, #0
 800228a:	75fb      	strb	r3, [r7, #23]
                }
                x -= (digit*place);
 800228c:	7bfb      	ldrb	r3, [r7, #15]
 800228e:	4618      	mov	r0, r3
 8002290:	f004 ff62 	bl	8007158 <__aeabi_i2f>
 8002294:	4603      	mov	r3, r0
 8002296:	4618      	mov	r0, r3
 8002298:	6939      	ldr	r1, [r7, #16]
 800229a:	f004 ffb1 	bl	8007200 <__aeabi_fmul>
 800229e:	4603      	mov	r3, r0
 80022a0:	6838      	ldr	r0, [r7, #0]
 80022a2:	4619      	mov	r1, r3
 80022a4:	f004 fea2 	bl	8006fec <__aeabi_fsub>
 80022a8:	4603      	mov	r3, r0
 80022aa:	603b      	str	r3, [r7, #0]
                place /= 10.0;
 80022ac:	6938      	ldr	r0, [r7, #16]
 80022ae:	490c      	ldr	r1, [pc, #48]	; (80022e0 <rprintfFloat+0x160>)
 80022b0:	f005 f85a 	bl	8007368 <__aeabi_fdiv>
 80022b4:	4603      	mov	r3, r0
 80022b6:	613b      	str	r3, [r7, #16]
                        break;
                else
                        place *= 10.0;
        }
        // print digits
        for(i=0; i<numDigits || sig; i++) {
 80022b8:	7dbb      	ldrb	r3, [r7, #22]
 80022ba:	f103 0301 	add.w	r3, r3, #1
 80022be:	75bb      	strb	r3, [r7, #22]
 80022c0:	7dba      	ldrb	r2, [r7, #22]
 80022c2:	79fb      	ldrb	r3, [r7, #7]
 80022c4:	429a      	cmp	r2, r3
 80022c6:	d3bc      	bcc.n	8002242 <rprintfFloat+0xc2>
 80022c8:	7dfb      	ldrb	r3, [r7, #23]
 80022ca:	2b00      	cmp	r3, #0
 80022cc:	d1b9      	bne.n	8002242 <rprintfFloat+0xc2>
                        sig=FALSE;
                }
                x -= (digit*place);
                place /= 10.0;
        }
}
 80022ce:	f107 071c 	add.w	r7, r7, #28
 80022d2:	46bd      	mov	sp, r7
 80022d4:	bd90      	pop	{r4, r7, pc}
 80022d6:	bf00      	nop
 80022d8:	3f800000 	.word	0x3f800000
 80022dc:	00000000 	.word	0x00000000
 80022e0:	41200000 	.word	0x41200000

080022e4 <rprintf2RamRom>:
// %c - character
// %s - strings
// and the width,precision,padding modifiers
// **this printf does not support floating point numbers
int rprintf2RamRom(const char *sfmt, ...)
{
 80022e4:	b40f      	push	{r0, r1, r2, r3}
 80022e6:	b580      	push	{r7, lr}
 80022e8:	b08e      	sub	sp, #56	; 0x38
 80022ea:	af00      	add	r7, sp, #0
	float v;
	long l;
	unsigned long u;
	int i;
	int fmt;
	unsigned char pad = ' ',f=0;
 80022ec:	f04f 0320 	mov.w	r3, #32
 80022f0:	f887 3027 	strb.w	r3, [r7, #39]	; 0x27
 80022f4:	f04f 0300 	mov.w	r3, #0
 80022f8:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
	int flush_left = 0, f_width = 0, prec = INF, hash = 0, do_long = 0;
 80022fc:	f04f 0300 	mov.w	r3, #0
 8002300:	623b      	str	r3, [r7, #32]
 8002302:	f04f 0300 	mov.w	r3, #0
 8002306:	61fb      	str	r3, [r7, #28]
 8002308:	f647 73fe 	movw	r3, #32766	; 0x7ffe
 800230c:	61bb      	str	r3, [r7, #24]
 800230e:	f04f 0300 	mov.w	r3, #0
 8002312:	617b      	str	r3, [r7, #20]
 8002314:	f04f 0300 	mov.w	r3, #0
 8002318:	613b      	str	r3, [r7, #16]
	int sign = 0;
 800231a:	f04f 0300 	mov.w	r3, #0
 800231e:	60fb      	str	r3, [r7, #12]
	va_list ap;
	va_start(ap, sfmt);
 8002320:	f107 0344 	add.w	r3, r7, #68	; 0x44
 8002324:	603b      	str	r3, [r7, #0]
	for (; sfmt[f]; f++){
 8002326:	e313      	b.n	8002950 <rprintf2RamRom+0x66c>
		if (sfmt[f] != '%'){	// not a format character
 8002328:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 800232c:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 800232e:	18d3      	adds	r3, r2, r3
 8002330:	781b      	ldrb	r3, [r3, #0]
 8002332:	2b25      	cmp	r3, #37	; 0x25
 8002334:	d008      	beq.n	8002348 <rprintf2RamRom+0x64>
			// then just output the char
			rprintfChar(sfmt[f]);
 8002336:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 800233a:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 800233c:	18d3      	adds	r3, r2, r3
 800233e:	781b      	ldrb	r3, [r3, #0]
 8002340:	4618      	mov	r0, r3
 8002342:	f7ff fd8f 	bl	8001e64 <rprintfChar>
 8002346:	e2fd      	b.n	8002944 <rprintf2RamRom+0x660>
		}
		else{
			f++;			// if we have a "%" then skip it
 8002348:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 800234c:	f103 0301 	add.w	r3, r3, #1
 8002350:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
			if (sfmt[f] == '-'){
 8002354:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002358:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 800235a:	18d3      	adds	r3, r2, r3
 800235c:	781b      	ldrb	r3, [r3, #0]
 800235e:	2b2d      	cmp	r3, #45	; 0x2d
 8002360:	d108      	bne.n	8002374 <rprintf2RamRom+0x90>
				flush_left = 1;	// minus: flush left
 8002362:	f04f 0301 	mov.w	r3, #1
 8002366:	623b      	str	r3, [r7, #32]
				f++;
 8002368:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 800236c:	f103 0301 	add.w	r3, r3, #1
 8002370:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
			}
            if (sfmt[f] == '0' || sfmt[f] == '.'){
 8002374:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002378:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 800237a:	18d3      	adds	r3, r2, r3
 800237c:	781b      	ldrb	r3, [r3, #0]
 800237e:	2b30      	cmp	r3, #48	; 0x30
 8002380:	d006      	beq.n	8002390 <rprintf2RamRom+0xac>
 8002382:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002386:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 8002388:	18d3      	adds	r3, r2, r3
 800238a:	781b      	ldrb	r3, [r3, #0]
 800238c:	2b2e      	cmp	r3, #46	; 0x2e
 800238e:	d109      	bne.n	80023a4 <rprintf2RamRom+0xc0>
					// padding with 0 rather than blank
					pad = '0';
 8002390:	f04f 0330 	mov.w	r3, #48	; 0x30
 8002394:	f887 3027 	strb.w	r3, [r7, #39]	; 0x27
					f++;
 8002398:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 800239c:	f103 0301 	add.w	r3, r3, #1
 80023a0:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
            }
            if (sfmt[f] == '*'){	// field width
 80023a4:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80023a8:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 80023aa:	18d3      	adds	r3, r2, r3
 80023ac:	781b      	ldrb	r3, [r3, #0]
 80023ae:	2b2a      	cmp	r3, #42	; 0x2a
 80023b0:	d10c      	bne.n	80023cc <rprintf2RamRom+0xe8>
					f_width = va_arg(ap, int);
 80023b2:	683b      	ldr	r3, [r7, #0]
 80023b4:	f103 0204 	add.w	r2, r3, #4
 80023b8:	603a      	str	r2, [r7, #0]
 80023ba:	681b      	ldr	r3, [r3, #0]
 80023bc:	61fb      	str	r3, [r7, #28]
					f++;
 80023be:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80023c2:	f103 0301 	add.w	r3, r3, #1
 80023c6:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
 80023ca:	e024      	b.n	8002416 <rprintf2RamRom+0x132>
            }
            else if (Isdigit(sfmt[f])){
 80023cc:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80023d0:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 80023d2:	18d3      	adds	r3, r2, r3
 80023d4:	781b      	ldrb	r3, [r3, #0]
 80023d6:	4618      	mov	r0, r3
 80023d8:	f000 fad0 	bl	800297c <Isdigit>
 80023dc:	4603      	mov	r3, r0
 80023de:	2b00      	cmp	r3, #0
 80023e0:	d019      	beq.n	8002416 <rprintf2RamRom+0x132>
					f_width = atoiRamRom((char *)&sfmt[f]);
 80023e2:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80023e6:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 80023e8:	18d3      	adds	r3, r2, r3
 80023ea:	4618      	mov	r0, r3
 80023ec:	f000 fadc 	bl	80029a8 <atoiRamRom>
 80023f0:	61f8      	str	r0, [r7, #28]
					while (Isdigit(sfmt[f]))
 80023f2:	e005      	b.n	8002400 <rprintf2RamRom+0x11c>
						f++;        // skip the digits
 80023f4:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80023f8:	f103 0301 	add.w	r3, r3, #1
 80023fc:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
					f_width = va_arg(ap, int);
					f++;
            }
            else if (Isdigit(sfmt[f])){
					f_width = atoiRamRom((char *)&sfmt[f]);
					while (Isdigit(sfmt[f]))
 8002400:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002404:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 8002406:	18d3      	adds	r3, r2, r3
 8002408:	781b      	ldrb	r3, [r3, #0]
 800240a:	4618      	mov	r0, r3
 800240c:	f000 fab6 	bl	800297c <Isdigit>
 8002410:	4603      	mov	r3, r0
 8002412:	2b00      	cmp	r3, #0
 8002414:	d1ee      	bne.n	80023f4 <rprintf2RamRom+0x110>
						f++;        // skip the digits
            }
            if (sfmt[f] == '.'){	// precision
 8002416:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 800241a:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 800241c:	18d3      	adds	r3, r2, r3
 800241e:	781b      	ldrb	r3, [r3, #0]
 8002420:	2b2e      	cmp	r3, #46	; 0x2e
 8002422:	d13e      	bne.n	80024a2 <rprintf2RamRom+0x1be>
					f++;
 8002424:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002428:	f103 0301 	add.w	r3, r3, #1
 800242c:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
					if (sfmt[f] == '*'){
 8002430:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002434:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 8002436:	18d3      	adds	r3, r2, r3
 8002438:	781b      	ldrb	r3, [r3, #0]
 800243a:	2b2a      	cmp	r3, #42	; 0x2a
 800243c:	d10c      	bne.n	8002458 <rprintf2RamRom+0x174>
						prec = va_arg(ap, int);
 800243e:	683b      	ldr	r3, [r7, #0]
 8002440:	f103 0204 	add.w	r2, r3, #4
 8002444:	603a      	str	r2, [r7, #0]
 8002446:	681b      	ldr	r3, [r3, #0]
 8002448:	61bb      	str	r3, [r7, #24]
						f++;
 800244a:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 800244e:	f103 0301 	add.w	r3, r3, #1
 8002452:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
 8002456:	e024      	b.n	80024a2 <rprintf2RamRom+0x1be>
					}
					else if (Isdigit(sfmt[f])){
 8002458:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 800245c:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 800245e:	18d3      	adds	r3, r2, r3
 8002460:	781b      	ldrb	r3, [r3, #0]
 8002462:	4618      	mov	r0, r3
 8002464:	f000 fa8a 	bl	800297c <Isdigit>
 8002468:	4603      	mov	r3, r0
 800246a:	2b00      	cmp	r3, #0
 800246c:	d019      	beq.n	80024a2 <rprintf2RamRom+0x1be>
						prec = atoiRamRom((char *)&sfmt[f]);
 800246e:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002472:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 8002474:	18d3      	adds	r3, r2, r3
 8002476:	4618      	mov	r0, r3
 8002478:	f000 fa96 	bl	80029a8 <atoiRamRom>
 800247c:	61b8      	str	r0, [r7, #24]
						while (Isdigit(sfmt[f]))
 800247e:	e005      	b.n	800248c <rprintf2RamRom+0x1a8>
							f++;    // skip the digits
 8002480:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002484:	f103 0301 	add.w	r3, r3, #1
 8002488:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
						prec = va_arg(ap, int);
						f++;
					}
					else if (Isdigit(sfmt[f])){
						prec = atoiRamRom((char *)&sfmt[f]);
						while (Isdigit(sfmt[f]))
 800248c:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002490:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 8002492:	18d3      	adds	r3, r2, r3
 8002494:	781b      	ldrb	r3, [r3, #0]
 8002496:	4618      	mov	r0, r3
 8002498:	f000 fa70 	bl	800297c <Isdigit>
 800249c:	4603      	mov	r3, r0
 800249e:	2b00      	cmp	r3, #0
 80024a0:	d1ee      	bne.n	8002480 <rprintf2RamRom+0x19c>
							f++;    // skip the digits
					}
				}
            if (sfmt[f] == '#'){	// alternate form
 80024a2:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80024a6:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 80024a8:	18d3      	adds	r3, r2, r3
 80024aa:	781b      	ldrb	r3, [r3, #0]
 80024ac:	2b23      	cmp	r3, #35	; 0x23
 80024ae:	d108      	bne.n	80024c2 <rprintf2RamRom+0x1de>
					hash = 1;
 80024b0:	f04f 0301 	mov.w	r3, #1
 80024b4:	617b      	str	r3, [r7, #20]
					f++;
 80024b6:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80024ba:	f103 0301 	add.w	r3, r3, #1
 80024be:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
            }
            if (sfmt[f] == 'l'){	// long format
 80024c2:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80024c6:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 80024c8:	18d3      	adds	r3, r2, r3
 80024ca:	781b      	ldrb	r3, [r3, #0]
 80024cc:	2b6c      	cmp	r3, #108	; 0x6c
 80024ce:	d108      	bne.n	80024e2 <rprintf2RamRom+0x1fe>
					do_long = 1;
 80024d0:	f04f 0301 	mov.w	r3, #1
 80024d4:	613b      	str	r3, [r7, #16]
					f++;
 80024d6:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80024da:	f103 0301 	add.w	r3, r3, #1
 80024de:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
            }

				fmt = sfmt[f];
 80024e2:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 80024e6:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 80024e8:	18d3      	adds	r3, r2, r3
 80024ea:	781b      	ldrb	r3, [r3, #0]
 80024ec:	60bb      	str	r3, [r7, #8]
				bp = buf;
 80024ee:	4b8f      	ldr	r3, [pc, #572]	; (800272c <rprintf2RamRom+0x448>)
 80024f0:	637b      	str	r3, [r7, #52]	; 0x34
				switch (fmt) {		// do the formatting
 80024f2:	68bb      	ldr	r3, [r7, #8]
 80024f4:	2b6f      	cmp	r3, #111	; 0x6f
 80024f6:	f000 80aa 	beq.w	800264e <rprintf2RamRom+0x36a>
 80024fa:	2b6f      	cmp	r3, #111	; 0x6f
 80024fc:	dc0e      	bgt.n	800251c <rprintf2RamRom+0x238>
 80024fe:	2b64      	cmp	r3, #100	; 0x64
 8002500:	d016      	beq.n	8002530 <rprintf2RamRom+0x24c>
 8002502:	2b64      	cmp	r3, #100	; 0x64
 8002504:	dc06      	bgt.n	8002514 <rprintf2RamRom+0x230>
 8002506:	2b25      	cmp	r3, #37	; 0x25
 8002508:	f000 81fc 	beq.w	8002904 <rprintf2RamRom+0x620>
 800250c:	2b63      	cmp	r3, #99	; 0x63
 800250e:	f000 8198 	beq.w	8002842 <rprintf2RamRom+0x55e>
 8002512:	e201      	b.n	8002918 <rprintf2RamRom+0x634>
 8002514:	2b66      	cmp	r3, #102	; 0x66
 8002516:	f000 8177 	beq.w	8002808 <rprintf2RamRom+0x524>
 800251a:	e1fd      	b.n	8002918 <rprintf2RamRom+0x634>
 800251c:	2b75      	cmp	r3, #117	; 0x75
 800251e:	f000 8096 	beq.w	800264e <rprintf2RamRom+0x36a>
 8002522:	2b78      	cmp	r3, #120	; 0x78
 8002524:	f000 8093 	beq.w	800264e <rprintf2RamRom+0x36a>
 8002528:	2b73      	cmp	r3, #115	; 0x73
 800252a:	f000 8196 	beq.w	800285a <rprintf2RamRom+0x576>
 800252e:	e1f3      	b.n	8002918 <rprintf2RamRom+0x634>
				case 'd':			// 'd' signed decimal
					if (do_long)
 8002530:	693b      	ldr	r3, [r7, #16]
 8002532:	2b00      	cmp	r3, #0
 8002534:	d006      	beq.n	8002544 <rprintf2RamRom+0x260>
						l = va_arg(ap, long);
 8002536:	683b      	ldr	r3, [r7, #0]
 8002538:	f103 0204 	add.w	r2, r3, #4
 800253c:	603a      	str	r2, [r7, #0]
 800253e:	681b      	ldr	r3, [r3, #0]
 8002540:	633b      	str	r3, [r7, #48]	; 0x30
 8002542:	e005      	b.n	8002550 <rprintf2RamRom+0x26c>
					else
						l = (long) (va_arg(ap, int));
 8002544:	683b      	ldr	r3, [r7, #0]
 8002546:	f103 0204 	add.w	r2, r3, #4
 800254a:	603a      	str	r2, [r7, #0]
 800254c:	681b      	ldr	r3, [r3, #0]
 800254e:	633b      	str	r3, [r7, #48]	; 0x30
					if (l < 0){
 8002550:	6b3b      	ldr	r3, [r7, #48]	; 0x30
 8002552:	2b00      	cmp	r3, #0
 8002554:	da06      	bge.n	8002564 <rprintf2RamRom+0x280>
						sign = 1;
 8002556:	f04f 0301 	mov.w	r3, #1
 800255a:	60fb      	str	r3, [r7, #12]
						l = -l;
 800255c:	6b3b      	ldr	r3, [r7, #48]	; 0x30
 800255e:	f1c3 0300 	rsb	r3, r3, #0
 8002562:	633b      	str	r3, [r7, #48]	; 0x30
					}
					do{
						*bp++ = l % 10 + '0';
 8002564:	6b39      	ldr	r1, [r7, #48]	; 0x30
 8002566:	4b72      	ldr	r3, [pc, #456]	; (8002730 <rprintf2RamRom+0x44c>)
 8002568:	fb83 2301 	smull	r2, r3, r3, r1
 800256c:	ea4f 02a3 	mov.w	r2, r3, asr #2
 8002570:	ea4f 73e1 	mov.w	r3, r1, asr #31
 8002574:	1ad2      	subs	r2, r2, r3
 8002576:	4613      	mov	r3, r2
 8002578:	ea4f 0383 	mov.w	r3, r3, lsl #2
 800257c:	189b      	adds	r3, r3, r2
 800257e:	ea4f 0343 	mov.w	r3, r3, lsl #1
 8002582:	1aca      	subs	r2, r1, r3
 8002584:	b2d3      	uxtb	r3, r2
 8002586:	f103 0330 	add.w	r3, r3, #48	; 0x30
 800258a:	b2da      	uxtb	r2, r3
 800258c:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800258e:	701a      	strb	r2, [r3, #0]
 8002590:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002592:	f103 0301 	add.w	r3, r3, #1
 8002596:	637b      	str	r3, [r7, #52]	; 0x34
					} while ((l /= 10) > 0);
 8002598:	6b3b      	ldr	r3, [r7, #48]	; 0x30
 800259a:	4a65      	ldr	r2, [pc, #404]	; (8002730 <rprintf2RamRom+0x44c>)
 800259c:	fb82 1203 	smull	r1, r2, r2, r3
 80025a0:	ea4f 02a2 	mov.w	r2, r2, asr #2
 80025a4:	ea4f 73e3 	mov.w	r3, r3, asr #31
 80025a8:	1ad3      	subs	r3, r2, r3
 80025aa:	633b      	str	r3, [r7, #48]	; 0x30
 80025ac:	6b3b      	ldr	r3, [r7, #48]	; 0x30
 80025ae:	2b00      	cmp	r3, #0
 80025b0:	dcd8      	bgt.n	8002564 <rprintf2RamRom+0x280>
					if (sign)
 80025b2:	68fb      	ldr	r3, [r7, #12]
 80025b4:	2b00      	cmp	r3, #0
 80025b6:	d007      	beq.n	80025c8 <rprintf2RamRom+0x2e4>
						*bp++ = '-';
 80025b8:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80025ba:	f04f 022d 	mov.w	r2, #45	; 0x2d
 80025be:	701a      	strb	r2, [r3, #0]
 80025c0:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80025c2:	f103 0301 	add.w	r3, r3, #1
 80025c6:	637b      	str	r3, [r7, #52]	; 0x34
					f_width = f_width - (bp - buf);
 80025c8:	4a58      	ldr	r2, [pc, #352]	; (800272c <rprintf2RamRom+0x448>)
 80025ca:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80025cc:	1ad3      	subs	r3, r2, r3
 80025ce:	69fa      	ldr	r2, [r7, #28]
 80025d0:	18d3      	adds	r3, r2, r3
 80025d2:	61fb      	str	r3, [r7, #28]
					if (!flush_left)
 80025d4:	6a3b      	ldr	r3, [r7, #32]
 80025d6:	2b00      	cmp	r3, #0
 80025d8:	d111      	bne.n	80025fe <rprintf2RamRom+0x31a>
						while (f_width-- > 0)
 80025da:	e004      	b.n	80025e6 <rprintf2RamRom+0x302>
							rprintfChar(pad);
 80025dc:	f897 3027 	ldrb.w	r3, [r7, #39]	; 0x27
 80025e0:	4618      	mov	r0, r3
 80025e2:	f7ff fc3f 	bl	8001e64 <rprintfChar>
					} while ((l /= 10) > 0);
					if (sign)
						*bp++ = '-';
					f_width = f_width - (bp - buf);
					if (!flush_left)
						while (f_width-- > 0)
 80025e6:	69fb      	ldr	r3, [r7, #28]
 80025e8:	2b00      	cmp	r3, #0
 80025ea:	bfd4      	ite	le
 80025ec:	2300      	movle	r3, #0
 80025ee:	2301      	movgt	r3, #1
 80025f0:	b2db      	uxtb	r3, r3
 80025f2:	69fa      	ldr	r2, [r7, #28]
 80025f4:	f102 32ff 	add.w	r2, r2, #4294967295
 80025f8:	61fa      	str	r2, [r7, #28]
 80025fa:	2b00      	cmp	r3, #0
 80025fc:	d1ee      	bne.n	80025dc <rprintf2RamRom+0x2f8>
							rprintfChar(pad);
					for (bp--; bp >= buf; bp--)
 80025fe:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002600:	f103 33ff 	add.w	r3, r3, #4294967295
 8002604:	637b      	str	r3, [r7, #52]	; 0x34
 8002606:	e008      	b.n	800261a <rprintf2RamRom+0x336>
						rprintfChar(*bp);
 8002608:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800260a:	781b      	ldrb	r3, [r3, #0]
 800260c:	4618      	mov	r0, r3
 800260e:	f7ff fc29 	bl	8001e64 <rprintfChar>
						*bp++ = '-';
					f_width = f_width - (bp - buf);
					if (!flush_left)
						while (f_width-- > 0)
							rprintfChar(pad);
					for (bp--; bp >= buf; bp--)
 8002612:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002614:	f103 33ff 	add.w	r3, r3, #4294967295
 8002618:	637b      	str	r3, [r7, #52]	; 0x34
 800261a:	6b7a      	ldr	r2, [r7, #52]	; 0x34
 800261c:	4b43      	ldr	r3, [pc, #268]	; (800272c <rprintf2RamRom+0x448>)
 800261e:	429a      	cmp	r2, r3
 8002620:	d2f2      	bcs.n	8002608 <rprintf2RamRom+0x324>
						rprintfChar(*bp);
					if (flush_left)
 8002622:	6a3b      	ldr	r3, [r7, #32]
 8002624:	2b00      	cmp	r3, #0
 8002626:	f000 8172 	beq.w	800290e <rprintf2RamRom+0x62a>
						while (f_width-- > 0)
 800262a:	e003      	b.n	8002634 <rprintf2RamRom+0x350>
							rprintfChar(' ');
 800262c:	f04f 0020 	mov.w	r0, #32
 8002630:	f7ff fc18 	bl	8001e64 <rprintfChar>
						while (f_width-- > 0)
							rprintfChar(pad);
					for (bp--; bp >= buf; bp--)
						rprintfChar(*bp);
					if (flush_left)
						while (f_width-- > 0)
 8002634:	69fb      	ldr	r3, [r7, #28]
 8002636:	2b00      	cmp	r3, #0
 8002638:	bfd4      	ite	le
 800263a:	2300      	movle	r3, #0
 800263c:	2301      	movgt	r3, #1
 800263e:	b2db      	uxtb	r3, r3
 8002640:	69fa      	ldr	r2, [r7, #28]
 8002642:	f102 32ff 	add.w	r2, r2, #4294967295
 8002646:	61fa      	str	r2, [r7, #28]
 8002648:	2b00      	cmp	r3, #0
 800264a:	d1ef      	bne.n	800262c <rprintf2RamRom+0x348>
							rprintfChar(' ');
					break;
 800264c:	e164      	b.n	8002918 <rprintf2RamRom+0x634>
            case 'o':			// 'o' octal number
            case 'x':			// 'x' hex number
            case 'u':			// 'u' unsigned decimal
					if (do_long)
 800264e:	693b      	ldr	r3, [r7, #16]
 8002650:	2b00      	cmp	r3, #0
 8002652:	d006      	beq.n	8002662 <rprintf2RamRom+0x37e>
						u = va_arg(ap, unsigned long);
 8002654:	683b      	ldr	r3, [r7, #0]
 8002656:	f103 0204 	add.w	r2, r3, #4
 800265a:	603a      	str	r2, [r7, #0]
 800265c:	681b      	ldr	r3, [r3, #0]
 800265e:	62fb      	str	r3, [r7, #44]	; 0x2c
 8002660:	e005      	b.n	800266e <rprintf2RamRom+0x38a>
					else
						u = (unsigned long) (va_arg(ap, unsigned));
 8002662:	683b      	ldr	r3, [r7, #0]
 8002664:	f103 0204 	add.w	r2, r3, #4
 8002668:	603a      	str	r2, [r7, #0]
 800266a:	681b      	ldr	r3, [r3, #0]
 800266c:	62fb      	str	r3, [r7, #44]	; 0x2c
					if (fmt == 'u'){// unsigned decimal
 800266e:	68bb      	ldr	r3, [r7, #8]
 8002670:	2b75      	cmp	r3, #117	; 0x75
 8002672:	d121      	bne.n	80026b8 <rprintf2RamRom+0x3d4>
						do {
							*bp++ = u % 10 + '0';
 8002674:	6af9      	ldr	r1, [r7, #44]	; 0x2c
 8002676:	4b2f      	ldr	r3, [pc, #188]	; (8002734 <rprintf2RamRom+0x450>)
 8002678:	fba3 2301 	umull	r2, r3, r3, r1
 800267c:	ea4f 02d3 	mov.w	r2, r3, lsr #3
 8002680:	4613      	mov	r3, r2
 8002682:	ea4f 0383 	mov.w	r3, r3, lsl #2
 8002686:	189b      	adds	r3, r3, r2
 8002688:	ea4f 0343 	mov.w	r3, r3, lsl #1
 800268c:	1aca      	subs	r2, r1, r3
 800268e:	b2d3      	uxtb	r3, r2
 8002690:	f103 0330 	add.w	r3, r3, #48	; 0x30
 8002694:	b2da      	uxtb	r2, r3
 8002696:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002698:	701a      	strb	r2, [r3, #0]
 800269a:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800269c:	f103 0301 	add.w	r3, r3, #1
 80026a0:	637b      	str	r3, [r7, #52]	; 0x34
						} while ((u /= 10) > 0);
 80026a2:	6afa      	ldr	r2, [r7, #44]	; 0x2c
 80026a4:	4b23      	ldr	r3, [pc, #140]	; (8002734 <rprintf2RamRom+0x450>)
 80026a6:	fba3 1302 	umull	r1, r3, r3, r2
 80026aa:	ea4f 03d3 	mov.w	r3, r3, lsr #3
 80026ae:	62fb      	str	r3, [r7, #44]	; 0x2c
 80026b0:	6afb      	ldr	r3, [r7, #44]	; 0x2c
 80026b2:	2b00      	cmp	r3, #0
 80026b4:	d1de      	bne.n	8002674 <rprintf2RamRom+0x390>
 80026b6:	e064      	b.n	8002782 <rprintf2RamRom+0x49e>
					}
					else if (fmt == 'o'){  // octal
 80026b8:	68bb      	ldr	r3, [r7, #8]
 80026ba:	2b6f      	cmp	r3, #111	; 0x6f
 80026bc:	d120      	bne.n	8002700 <rprintf2RamRom+0x41c>
						do {
							*bp++ = u % 8 + '0';
 80026be:	6afb      	ldr	r3, [r7, #44]	; 0x2c
 80026c0:	b2db      	uxtb	r3, r3
 80026c2:	f003 0307 	and.w	r3, r3, #7
 80026c6:	b2db      	uxtb	r3, r3
 80026c8:	f103 0330 	add.w	r3, r3, #48	; 0x30
 80026cc:	b2da      	uxtb	r2, r3
 80026ce:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80026d0:	701a      	strb	r2, [r3, #0]
 80026d2:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80026d4:	f103 0301 	add.w	r3, r3, #1
 80026d8:	637b      	str	r3, [r7, #52]	; 0x34
						} while ((u /= 8) > 0);
 80026da:	6afb      	ldr	r3, [r7, #44]	; 0x2c
 80026dc:	ea4f 03d3 	mov.w	r3, r3, lsr #3
 80026e0:	62fb      	str	r3, [r7, #44]	; 0x2c
 80026e2:	6afb      	ldr	r3, [r7, #44]	; 0x2c
 80026e4:	2b00      	cmp	r3, #0
 80026e6:	d1ea      	bne.n	80026be <rprintf2RamRom+0x3da>
						if (hash)
 80026e8:	697b      	ldr	r3, [r7, #20]
 80026ea:	2b00      	cmp	r3, #0
 80026ec:	d049      	beq.n	8002782 <rprintf2RamRom+0x49e>
							*bp++ = '0';
 80026ee:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80026f0:	f04f 0230 	mov.w	r2, #48	; 0x30
 80026f4:	701a      	strb	r2, [r3, #0]
 80026f6:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80026f8:	f103 0301 	add.w	r3, r3, #1
 80026fc:	637b      	str	r3, [r7, #52]	; 0x34
 80026fe:	e040      	b.n	8002782 <rprintf2RamRom+0x49e>
					}
					else if (fmt == 'x'){	// hex
 8002700:	68bb      	ldr	r3, [r7, #8]
 8002702:	2b78      	cmp	r3, #120	; 0x78
 8002704:	d13d      	bne.n	8002782 <rprintf2RamRom+0x49e>
						do {
							i = u % 16;
 8002706:	6afb      	ldr	r3, [r7, #44]	; 0x2c
 8002708:	f003 030f 	and.w	r3, r3, #15
 800270c:	62bb      	str	r3, [r7, #40]	; 0x28
							if (i < 10)
 800270e:	6abb      	ldr	r3, [r7, #40]	; 0x28
 8002710:	2b09      	cmp	r3, #9
 8002712:	dc11      	bgt.n	8002738 <rprintf2RamRom+0x454>
								*bp++ = i + '0';
 8002714:	6abb      	ldr	r3, [r7, #40]	; 0x28
 8002716:	b2db      	uxtb	r3, r3
 8002718:	f103 0330 	add.w	r3, r3, #48	; 0x30
 800271c:	b2da      	uxtb	r2, r3
 800271e:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002720:	701a      	strb	r2, [r3, #0]
 8002722:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002724:	f103 0301 	add.w	r3, r3, #1
 8002728:	637b      	str	r3, [r7, #52]	; 0x34
 800272a:	e010      	b.n	800274e <rprintf2RamRom+0x46a>
 800272c:	200001b0 	.word	0x200001b0
 8002730:	66666667 	.word	0x66666667
 8002734:	cccccccd 	.word	0xcccccccd
							else
								*bp++ = i - 10 + 'a';
 8002738:	6abb      	ldr	r3, [r7, #40]	; 0x28
 800273a:	b2db      	uxtb	r3, r3
 800273c:	f103 0357 	add.w	r3, r3, #87	; 0x57
 8002740:	b2da      	uxtb	r2, r3
 8002742:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002744:	701a      	strb	r2, [r3, #0]
 8002746:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002748:	f103 0301 	add.w	r3, r3, #1
 800274c:	637b      	str	r3, [r7, #52]	; 0x34
						} while ((u /= 16) > 0);
 800274e:	6afb      	ldr	r3, [r7, #44]	; 0x2c
 8002750:	ea4f 1313 	mov.w	r3, r3, lsr #4
 8002754:	62fb      	str	r3, [r7, #44]	; 0x2c
 8002756:	6afb      	ldr	r3, [r7, #44]	; 0x2c
 8002758:	2b00      	cmp	r3, #0
 800275a:	d1d4      	bne.n	8002706 <rprintf2RamRom+0x422>
						if (hash){
 800275c:	697b      	ldr	r3, [r7, #20]
 800275e:	2b00      	cmp	r3, #0
 8002760:	d00f      	beq.n	8002782 <rprintf2RamRom+0x49e>
							*bp++ = 'x';
 8002762:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002764:	f04f 0278 	mov.w	r2, #120	; 0x78
 8002768:	701a      	strb	r2, [r3, #0]
 800276a:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800276c:	f103 0301 	add.w	r3, r3, #1
 8002770:	637b      	str	r3, [r7, #52]	; 0x34
							*bp++ = '0';
 8002772:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002774:	f04f 0230 	mov.w	r2, #48	; 0x30
 8002778:	701a      	strb	r2, [r3, #0]
 800277a:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 800277c:	f103 0301 	add.w	r3, r3, #1
 8002780:	637b      	str	r3, [r7, #52]	; 0x34
						}
					}
					i = f_width - (bp - buf);
 8002782:	4a7c      	ldr	r2, [pc, #496]	; (8002974 <rprintf2RamRom+0x690>)
 8002784:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002786:	1ad2      	subs	r2, r2, r3
 8002788:	69fb      	ldr	r3, [r7, #28]
 800278a:	18d3      	adds	r3, r2, r3
 800278c:	62bb      	str	r3, [r7, #40]	; 0x28
					if (!flush_left)
 800278e:	6a3b      	ldr	r3, [r7, #32]
 8002790:	2b00      	cmp	r3, #0
 8002792:	d111      	bne.n	80027b8 <rprintf2RamRom+0x4d4>
						while (i-- > 0)
 8002794:	e004      	b.n	80027a0 <rprintf2RamRom+0x4bc>
							rprintfChar(pad);
 8002796:	f897 3027 	ldrb.w	r3, [r7, #39]	; 0x27
 800279a:	4618      	mov	r0, r3
 800279c:	f7ff fb62 	bl	8001e64 <rprintfChar>
							*bp++ = '0';
						}
					}
					i = f_width - (bp - buf);
					if (!flush_left)
						while (i-- > 0)
 80027a0:	6abb      	ldr	r3, [r7, #40]	; 0x28
 80027a2:	2b00      	cmp	r3, #0
 80027a4:	bfd4      	ite	le
 80027a6:	2300      	movle	r3, #0
 80027a8:	2301      	movgt	r3, #1
 80027aa:	b2db      	uxtb	r3, r3
 80027ac:	6aba      	ldr	r2, [r7, #40]	; 0x28
 80027ae:	f102 32ff 	add.w	r2, r2, #4294967295
 80027b2:	62ba      	str	r2, [r7, #40]	; 0x28
 80027b4:	2b00      	cmp	r3, #0
 80027b6:	d1ee      	bne.n	8002796 <rprintf2RamRom+0x4b2>
							rprintfChar(pad);
					for (bp--; bp >= buf; bp--)
 80027b8:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80027ba:	f103 33ff 	add.w	r3, r3, #4294967295
 80027be:	637b      	str	r3, [r7, #52]	; 0x34
 80027c0:	e008      	b.n	80027d4 <rprintf2RamRom+0x4f0>
						rprintfChar((int) (*bp));
 80027c2:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80027c4:	781b      	ldrb	r3, [r3, #0]
 80027c6:	4618      	mov	r0, r3
 80027c8:	f7ff fb4c 	bl	8001e64 <rprintfChar>
					}
					i = f_width - (bp - buf);
					if (!flush_left)
						while (i-- > 0)
							rprintfChar(pad);
					for (bp--; bp >= buf; bp--)
 80027cc:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80027ce:	f103 33ff 	add.w	r3, r3, #4294967295
 80027d2:	637b      	str	r3, [r7, #52]	; 0x34
 80027d4:	6b7a      	ldr	r2, [r7, #52]	; 0x34
 80027d6:	4b67      	ldr	r3, [pc, #412]	; (8002974 <rprintf2RamRom+0x690>)
 80027d8:	429a      	cmp	r2, r3
 80027da:	d2f2      	bcs.n	80027c2 <rprintf2RamRom+0x4de>
						rprintfChar((int) (*bp));
					if (flush_left)
 80027dc:	6a3b      	ldr	r3, [r7, #32]
 80027de:	2b00      	cmp	r3, #0
 80027e0:	f000 8097 	beq.w	8002912 <rprintf2RamRom+0x62e>
						while (i-- > 0)
 80027e4:	e003      	b.n	80027ee <rprintf2RamRom+0x50a>
							rprintfChar(' ');
 80027e6:	f04f 0020 	mov.w	r0, #32
 80027ea:	f7ff fb3b 	bl	8001e64 <rprintfChar>
						while (i-- > 0)
							rprintfChar(pad);
					for (bp--; bp >= buf; bp--)
						rprintfChar((int) (*bp));
					if (flush_left)
						while (i-- > 0)
 80027ee:	6abb      	ldr	r3, [r7, #40]	; 0x28
 80027f0:	2b00      	cmp	r3, #0
 80027f2:	bfd4      	ite	le
 80027f4:	2300      	movle	r3, #0
 80027f6:	2301      	movgt	r3, #1
 80027f8:	b2db      	uxtb	r3, r3
 80027fa:	6aba      	ldr	r2, [r7, #40]	; 0x28
 80027fc:	f102 32ff 	add.w	r2, r2, #4294967295
 8002800:	62ba      	str	r2, [r7, #40]	; 0x28
 8002802:	2b00      	cmp	r3, #0
 8002804:	d1ef      	bne.n	80027e6 <rprintf2RamRom+0x502>
							rprintfChar(' ');
					break;
 8002806:	e087      	b.n	8002918 <rprintf2RamRom+0x634>
	    case 'f':			// 'f' character
					v = (float) (va_arg(ap, double));
 8002808:	683b      	ldr	r3, [r7, #0]
 800280a:	f103 0307 	add.w	r3, r3, #7
 800280e:	f023 0307 	bic.w	r3, r3, #7
 8002812:	f103 0208 	add.w	r2, r3, #8
 8002816:	603a      	str	r2, [r7, #0]
 8002818:	e9d3 2300 	ldrd	r2, r3, [r3]
 800281c:	4610      	mov	r0, r2
 800281e:	4619      	mov	r1, r3
 8002820:	f004 fb90 	bl	8006f44 <__aeabi_d2f>
 8002824:	4603      	mov	r3, r0
 8002826:	607b      	str	r3, [r7, #4]
					if(!f_width)f_width=9;//default width
 8002828:	69fb      	ldr	r3, [r7, #28]
 800282a:	2b00      	cmp	r3, #0
 800282c:	d102      	bne.n	8002834 <rprintf2RamRom+0x550>
 800282e:	f04f 0309 	mov.w	r3, #9
 8002832:	61fb      	str	r3, [r7, #28]
					rprintfFloat(f_width,v);//f_width
 8002834:	69fb      	ldr	r3, [r7, #28]
 8002836:	b2db      	uxtb	r3, r3
 8002838:	4618      	mov	r0, r3
 800283a:	6879      	ldr	r1, [r7, #4]
 800283c:	f7ff fca0 	bl	8002180 <rprintfFloat>
					break;
 8002840:	e06a      	b.n	8002918 <rprintf2RamRom+0x634>
            case 'c':			// 'c' character
					i = va_arg(ap, int);
 8002842:	683b      	ldr	r3, [r7, #0]
 8002844:	f103 0204 	add.w	r2, r3, #4
 8002848:	603a      	str	r2, [r7, #0]
 800284a:	681b      	ldr	r3, [r3, #0]
 800284c:	62bb      	str	r3, [r7, #40]	; 0x28
					rprintfChar((int) (i));
 800284e:	6abb      	ldr	r3, [r7, #40]	; 0x28
 8002850:	b2db      	uxtb	r3, r3
 8002852:	4618      	mov	r0, r3
 8002854:	f7ff fb06 	bl	8001e64 <rprintfChar>
					break;
 8002858:	e05e      	b.n	8002918 <rprintf2RamRom+0x634>
            case 's':			// 's' string
					bp = va_arg(ap, unsigned char *);
 800285a:	683b      	ldr	r3, [r7, #0]
 800285c:	f103 0204 	add.w	r2, r3, #4
 8002860:	603a      	str	r2, [r7, #0]
 8002862:	681b      	ldr	r3, [r3, #0]
 8002864:	637b      	str	r3, [r7, #52]	; 0x34
					if (!bp)
 8002866:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 8002868:	2b00      	cmp	r3, #0
 800286a:	d101      	bne.n	8002870 <rprintf2RamRom+0x58c>
						bp = (unsigned char *) "(nil)";
 800286c:	4b42      	ldr	r3, [pc, #264]	; (8002978 <rprintf2RamRom+0x694>)
 800286e:	637b      	str	r3, [r7, #52]	; 0x34
					f_width = f_width - rStrLen((char *) bp);
 8002870:	6b78      	ldr	r0, [r7, #52]	; 0x34
 8002872:	f000 f8c9 	bl	8002a08 <rStrLen>
 8002876:	4603      	mov	r3, r0
 8002878:	69fa      	ldr	r2, [r7, #28]
 800287a:	1ad3      	subs	r3, r2, r3
 800287c:	61fb      	str	r3, [r7, #28]
					if (!flush_left)
 800287e:	6a3b      	ldr	r3, [r7, #32]
 8002880:	2b00      	cmp	r3, #0
 8002882:	d111      	bne.n	80028a8 <rprintf2RamRom+0x5c4>
						while (f_width-- > 0)
 8002884:	e004      	b.n	8002890 <rprintf2RamRom+0x5ac>
							rprintfChar(pad);
 8002886:	f897 3027 	ldrb.w	r3, [r7, #39]	; 0x27
 800288a:	4618      	mov	r0, r3
 800288c:	f7ff faea 	bl	8001e64 <rprintfChar>
					bp = va_arg(ap, unsigned char *);
					if (!bp)
						bp = (unsigned char *) "(nil)";
					f_width = f_width - rStrLen((char *) bp);
					if (!flush_left)
						while (f_width-- > 0)
 8002890:	69fb      	ldr	r3, [r7, #28]
 8002892:	2b00      	cmp	r3, #0
 8002894:	bfd4      	ite	le
 8002896:	2300      	movle	r3, #0
 8002898:	2301      	movgt	r3, #1
 800289a:	b2db      	uxtb	r3, r3
 800289c:	69fa      	ldr	r2, [r7, #28]
 800289e:	f102 32ff 	add.w	r2, r2, #4294967295
 80028a2:	61fa      	str	r2, [r7, #28]
 80028a4:	2b00      	cmp	r3, #0
 80028a6:	d1ee      	bne.n	8002886 <rprintf2RamRom+0x5a2>
							rprintfChar(pad);
					for (i = 0; *bp && i < prec; i++){
 80028a8:	f04f 0300 	mov.w	r3, #0
 80028ac:	62bb      	str	r3, [r7, #40]	; 0x28
 80028ae:	e00c      	b.n	80028ca <rprintf2RamRom+0x5e6>
						rprintfChar(*bp);
 80028b0:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80028b2:	781b      	ldrb	r3, [r3, #0]
 80028b4:	4618      	mov	r0, r3
 80028b6:	f7ff fad5 	bl	8001e64 <rprintfChar>
						bp++;
 80028ba:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80028bc:	f103 0301 	add.w	r3, r3, #1
 80028c0:	637b      	str	r3, [r7, #52]	; 0x34
						bp = (unsigned char *) "(nil)";
					f_width = f_width - rStrLen((char *) bp);
					if (!flush_left)
						while (f_width-- > 0)
							rprintfChar(pad);
					for (i = 0; *bp && i < prec; i++){
 80028c2:	6abb      	ldr	r3, [r7, #40]	; 0x28
 80028c4:	f103 0301 	add.w	r3, r3, #1
 80028c8:	62bb      	str	r3, [r7, #40]	; 0x28
 80028ca:	6b7b      	ldr	r3, [r7, #52]	; 0x34
 80028cc:	781b      	ldrb	r3, [r3, #0]
 80028ce:	2b00      	cmp	r3, #0
 80028d0:	d003      	beq.n	80028da <rprintf2RamRom+0x5f6>
 80028d2:	6aba      	ldr	r2, [r7, #40]	; 0x28
 80028d4:	69bb      	ldr	r3, [r7, #24]
 80028d6:	429a      	cmp	r2, r3
 80028d8:	dbea      	blt.n	80028b0 <rprintf2RamRom+0x5cc>
						rprintfChar(*bp);
						bp++;
					}
					if (flush_left)
 80028da:	6a3b      	ldr	r3, [r7, #32]
 80028dc:	2b00      	cmp	r3, #0
 80028de:	d01a      	beq.n	8002916 <rprintf2RamRom+0x632>
						while (f_width-- > 0)
 80028e0:	e003      	b.n	80028ea <rprintf2RamRom+0x606>
							rprintfChar(' ');
 80028e2:	f04f 0020 	mov.w	r0, #32
 80028e6:	f7ff fabd 	bl	8001e64 <rprintfChar>
					for (i = 0; *bp && i < prec; i++){
						rprintfChar(*bp);
						bp++;
					}
					if (flush_left)
						while (f_width-- > 0)
 80028ea:	69fb      	ldr	r3, [r7, #28]
 80028ec:	2b00      	cmp	r3, #0
 80028ee:	bfd4      	ite	le
 80028f0:	2300      	movle	r3, #0
 80028f2:	2301      	movgt	r3, #1
 80028f4:	b2db      	uxtb	r3, r3
 80028f6:	69fa      	ldr	r2, [r7, #28]
 80028f8:	f102 32ff 	add.w	r2, r2, #4294967295
 80028fc:	61fa      	str	r2, [r7, #28]
 80028fe:	2b00      	cmp	r3, #0
 8002900:	d1ef      	bne.n	80028e2 <rprintf2RamRom+0x5fe>
							rprintfChar(' ');
					break;
 8002902:	e009      	b.n	8002918 <rprintf2RamRom+0x634>
            case '%':			// '%' character
					rprintfChar('%');
 8002904:	f04f 0025 	mov.w	r0, #37	; 0x25
 8002908:	f7ff faac 	bl	8001e64 <rprintfChar>
					break;
 800290c:	e004      	b.n	8002918 <rprintf2RamRom+0x634>
					for (bp--; bp >= buf; bp--)
						rprintfChar(*bp);
					if (flush_left)
						while (f_width-- > 0)
							rprintfChar(' ');
					break;
 800290e:	bf00      	nop
 8002910:	e002      	b.n	8002918 <rprintf2RamRom+0x634>
					for (bp--; bp >= buf; bp--)
						rprintfChar((int) (*bp));
					if (flush_left)
						while (i-- > 0)
							rprintfChar(' ');
					break;
 8002912:	bf00      	nop
 8002914:	e000      	b.n	8002918 <rprintf2RamRom+0x634>
						bp++;
					}
					if (flush_left)
						while (f_width-- > 0)
							rprintfChar(' ');
					break;
 8002916:	bf00      	nop
            case '%':			// '%' character
					rprintfChar('%');
					break;
			}
			flush_left = 0, f_width = 0, prec = INF, hash = 0, do_long = 0;
 8002918:	f04f 0300 	mov.w	r3, #0
 800291c:	623b      	str	r3, [r7, #32]
 800291e:	f04f 0300 	mov.w	r3, #0
 8002922:	61fb      	str	r3, [r7, #28]
 8002924:	f647 73fe 	movw	r3, #32766	; 0x7ffe
 8002928:	61bb      	str	r3, [r7, #24]
 800292a:	f04f 0300 	mov.w	r3, #0
 800292e:	617b      	str	r3, [r7, #20]
 8002930:	f04f 0300 	mov.w	r3, #0
 8002934:	613b      	str	r3, [r7, #16]
			sign = 0;
 8002936:	f04f 0300 	mov.w	r3, #0
 800293a:	60fb      	str	r3, [r7, #12]
			pad = ' ';
 800293c:	f04f 0320 	mov.w	r3, #32
 8002940:	f887 3027 	strb.w	r3, [r7, #39]	; 0x27
	unsigned char pad = ' ',f=0;
	int flush_left = 0, f_width = 0, prec = INF, hash = 0, do_long = 0;
	int sign = 0;
	va_list ap;
	va_start(ap, sfmt);
	for (; sfmt[f]; f++){
 8002944:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002948:	f103 0301 	add.w	r3, r3, #1
 800294c:	f887 3026 	strb.w	r3, [r7, #38]	; 0x26
 8002950:	f897 3026 	ldrb.w	r3, [r7, #38]	; 0x26
 8002954:	6c3a      	ldr	r2, [r7, #64]	; 0x40
 8002956:	18d3      	adds	r3, r2, r3
 8002958:	781b      	ldrb	r3, [r3, #0]
 800295a:	2b00      	cmp	r3, #0
 800295c:	f47f ace4 	bne.w	8002328 <rprintf2RamRom+0x44>
			sign = 0;
			pad = ' ';
		}
	}
	va_end(ap);
	return 0;
 8002960:	f04f 0300 	mov.w	r3, #0
}
 8002964:	4618      	mov	r0, r3
 8002966:	f107 0738 	add.w	r7, r7, #56	; 0x38
 800296a:	46bd      	mov	sp, r7
 800296c:	e8bd 4080 	ldmia.w	sp!, {r7, lr}
 8002970:	b004      	add	sp, #16
 8002972:	4770      	bx	lr
 8002974:	200001b0 	.word	0x200001b0
 8002978:	0800775c 	.word	0x0800775c

0800297c <Isdigit>:

unsigned char Isdigit(char c) {
 800297c:	b480      	push	{r7}
 800297e:	b083      	sub	sp, #12
 8002980:	af00      	add	r7, sp, #0
 8002982:	4603      	mov	r3, r0
 8002984:	71fb      	strb	r3, [r7, #7]
	if((c >= 0x30) && (c <= 0x39))
 8002986:	79fb      	ldrb	r3, [r7, #7]
 8002988:	2b2f      	cmp	r3, #47	; 0x2f
 800298a:	d905      	bls.n	8002998 <Isdigit+0x1c>
 800298c:	79fb      	ldrb	r3, [r7, #7]
 800298e:	2b39      	cmp	r3, #57	; 0x39
 8002990:	d802      	bhi.n	8002998 <Isdigit+0x1c>
		return TRUE;
 8002992:	f04f 0301 	mov.w	r3, #1
 8002996:	e001      	b.n	800299c <Isdigit+0x20>
	else
		return FALSE;
 8002998:	f04f 0300 	mov.w	r3, #0
}
 800299c:	4618      	mov	r0, r3
 800299e:	f107 070c 	add.w	r7, r7, #12
 80029a2:	46bd      	mov	sp, r7
 80029a4:	bc80      	pop	{r7}
 80029a6:	4770      	bx	lr

080029a8 <atoiRamRom>:

int atoiRamRom(char *str) {
 80029a8:	b580      	push	{r7, lr}
 80029aa:	b084      	sub	sp, #16
 80029ac:	af00      	add	r7, sp, #0
 80029ae:	6078      	str	r0, [r7, #4]
	int num = 0;unsigned char r=0;
 80029b0:	f04f 0300 	mov.w	r3, #0
 80029b4:	60fb      	str	r3, [r7, #12]
 80029b6:	f04f 0300 	mov.w	r3, #0
 80029ba:	72fb      	strb	r3, [r7, #11]
	while(Isdigit(str[r])){
 80029bc:	e014      	b.n	80029e8 <atoiRamRom+0x40>
		num *= 10;
 80029be:	68fa      	ldr	r2, [r7, #12]
 80029c0:	4613      	mov	r3, r2
 80029c2:	ea4f 0383 	mov.w	r3, r3, lsl #2
 80029c6:	189b      	adds	r3, r3, r2
 80029c8:	ea4f 0343 	mov.w	r3, r3, lsl #1
 80029cc:	60fb      	str	r3, [r7, #12]
		num += str[r++] - 0x30;
 80029ce:	7afb      	ldrb	r3, [r7, #11]
 80029d0:	687a      	ldr	r2, [r7, #4]
 80029d2:	18d3      	adds	r3, r2, r3
 80029d4:	781b      	ldrb	r3, [r3, #0]
 80029d6:	f1a3 0330 	sub.w	r3, r3, #48	; 0x30
 80029da:	68fa      	ldr	r2, [r7, #12]
 80029dc:	18d3      	adds	r3, r2, r3
 80029de:	60fb      	str	r3, [r7, #12]
 80029e0:	7afb      	ldrb	r3, [r7, #11]
 80029e2:	f103 0301 	add.w	r3, r3, #1
 80029e6:	72fb      	strb	r3, [r7, #11]
		return FALSE;
}

int atoiRamRom(char *str) {
	int num = 0;unsigned char r=0;
	while(Isdigit(str[r])){
 80029e8:	7afb      	ldrb	r3, [r7, #11]
 80029ea:	687a      	ldr	r2, [r7, #4]
 80029ec:	18d3      	adds	r3, r2, r3
 80029ee:	781b      	ldrb	r3, [r3, #0]
 80029f0:	4618      	mov	r0, r3
 80029f2:	f7ff ffc3 	bl	800297c <Isdigit>
 80029f6:	4603      	mov	r3, r0
 80029f8:	2b00      	cmp	r3, #0
 80029fa:	d1e0      	bne.n	80029be <atoiRamRom+0x16>
		num *= 10;
		num += str[r++] - 0x30;
	}
	return num;
 80029fc:	68fb      	ldr	r3, [r7, #12]
}
 80029fe:	4618      	mov	r0, r3
 8002a00:	f107 0710 	add.w	r7, r7, #16
 8002a04:	46bd      	mov	sp, r7
 8002a06:	bd80      	pop	{r7, pc}

08002a08 <rStrLen>:

int rStrLen(char *str) {
 8002a08:	b480      	push	{r7}
 8002a0a:	b085      	sub	sp, #20
 8002a0c:	af00      	add	r7, sp, #0
 8002a0e:	6078      	str	r0, [r7, #4]
	unsigned char i=0;
 8002a10:	f04f 0300 	mov.w	r3, #0
 8002a14:	73fb      	strb	r3, [r7, #15]
	while(str[i++]);
 8002a16:	bf00      	nop
 8002a18:	7bfb      	ldrb	r3, [r7, #15]
 8002a1a:	687a      	ldr	r2, [r7, #4]
 8002a1c:	18d3      	adds	r3, r2, r3
 8002a1e:	781b      	ldrb	r3, [r3, #0]
 8002a20:	2b00      	cmp	r3, #0
 8002a22:	bf0c      	ite	eq
 8002a24:	2300      	moveq	r3, #0
 8002a26:	2301      	movne	r3, #1
 8002a28:	b2db      	uxtb	r3, r3
 8002a2a:	7bfa      	ldrb	r2, [r7, #15]
 8002a2c:	f102 0201 	add.w	r2, r2, #1
 8002a30:	73fa      	strb	r2, [r7, #15]
 8002a32:	2b00      	cmp	r3, #0
 8002a34:	d1f0      	bne.n	8002a18 <rStrLen+0x10>
	return (int) i;
 8002a36:	7bfb      	ldrb	r3, [r7, #15]
}
 8002a38:	4618      	mov	r0, r3
 8002a3a:	f107 0714 	add.w	r7, r7, #20
 8002a3e:	46bd      	mov	sp, r7
 8002a40:	bc80      	pop	{r7}
 8002a42:	4770      	bx	lr

08002a44 <SD_LowLevel_DeInit>:
  * @brief  DeInitializes the SD/SD communication.
  * @param  None
  * @retval None
  */
void SD_LowLevel_DeInit(void)
{
 8002a44:	b580      	push	{r7, lr}
 8002a46:	b082      	sub	sp, #8
 8002a48:	af00      	add	r7, sp, #0
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  SPI_Cmd(SD_SPI, DISABLE); /*!< SD_SPI disable */
 8002a4a:	481c      	ldr	r0, [pc, #112]	; (8002abc <SD_LowLevel_DeInit+0x78>)
 8002a4c:	f04f 0100 	mov.w	r1, #0
 8002a50:	f002 f8fb 	bl	8004c4a <SPI_Cmd>
  SPI_I2S_DeInit(SD_SPI);   /*!< DeInitializes the SD_SPI */
 8002a54:	4819      	ldr	r0, [pc, #100]	; (8002abc <SD_LowLevel_DeInit+0x78>)
 8002a56:	f002 f841 	bl	8004adc <SPI_I2S_DeInit>
  
  /*!< SD_SPI Periph clock disable */
  RCC_APB2PeriphClockCmd(SD_SPI_CLK, DISABLE); 
 8002a5a:	f44f 5080 	mov.w	r0, #4096	; 0x1000
 8002a5e:	f04f 0100 	mov.w	r1, #0
 8002a62:	f001 ffb7 	bl	80049d4 <RCC_APB2PeriphClockCmd>
  
  /*!< Configure SD_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_SCK_PIN;
 8002a66:	f04f 0320 	mov.w	r3, #32
 8002a6a:	80bb      	strh	r3, [r7, #4]
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
 8002a6c:	f04f 0304 	mov.w	r3, #4
 8002a70:	71fb      	strb	r3, [r7, #7]
  GPIO_Init(SD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
 8002a72:	f107 0304 	add.w	r3, r7, #4
 8002a76:	4812      	ldr	r0, [pc, #72]	; (8002ac0 <SD_LowLevel_DeInit+0x7c>)
 8002a78:	4619      	mov	r1, r3
 8002a7a:	f001 fd3e 	bl	80044fa <GPIO_Init>

  /*!< Configure SD_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_MISO_PIN;
 8002a7e:	f04f 0340 	mov.w	r3, #64	; 0x40
 8002a82:	80bb      	strh	r3, [r7, #4]
  GPIO_Init(SD_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
 8002a84:	f107 0304 	add.w	r3, r7, #4
 8002a88:	480d      	ldr	r0, [pc, #52]	; (8002ac0 <SD_LowLevel_DeInit+0x7c>)
 8002a8a:	4619      	mov	r1, r3
 8002a8c:	f001 fd35 	bl	80044fa <GPIO_Init>

  /*!< Configure SD_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_MOSI_PIN;
 8002a90:	f04f 0380 	mov.w	r3, #128	; 0x80
 8002a94:	80bb      	strh	r3, [r7, #4]
  GPIO_Init(SD_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
 8002a96:	f107 0304 	add.w	r3, r7, #4
 8002a9a:	4809      	ldr	r0, [pc, #36]	; (8002ac0 <SD_LowLevel_DeInit+0x7c>)
 8002a9c:	4619      	mov	r1, r3
 8002a9e:	f001 fd2c 	bl	80044fa <GPIO_Init>

  /*!< Configure SD_SPI_CS_PIN pin: SD Card CS pin */
  GPIO_InitStructure.GPIO_Pin = SD_CS_PIN;
 8002aa2:	f44f 4380 	mov.w	r3, #16384	; 0x4000
 8002aa6:	80bb      	strh	r3, [r7, #4]
  GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStructure);
 8002aa8:	f107 0304 	add.w	r3, r7, #4
 8002aac:	4805      	ldr	r0, [pc, #20]	; (8002ac4 <SD_LowLevel_DeInit+0x80>)
 8002aae:	4619      	mov	r1, r3
 8002ab0:	f001 fd23 	bl	80044fa <GPIO_Init>

}
 8002ab4:	f107 0708 	add.w	r7, r7, #8
 8002ab8:	46bd      	mov	sp, r7
 8002aba:	bd80      	pop	{r7, pc}
 8002abc:	40013000 	.word	0x40013000
 8002ac0:	40010800 	.word	0x40010800
 8002ac4:	40010c00 	.word	0x40010c00

08002ac8 <SD_LowLevel_Init>:
  * @brief  Initializes the SD_SPI and CS pins.
  * @param  None
  * @retval None
  */
void SD_LowLevel_Init(void)
{
 8002ac8:	b580      	push	{r7, lr}
 8002aca:	b086      	sub	sp, #24
 8002acc:	af00      	add	r7, sp, #0
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;

  /*!< SD_SPI_CS_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_MISO_GPIO, SD_SPI_DETECT_GPIO 
       and SD_SPI_SCK_GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(SD_CS_GPIO_CLK | SD_SPI_MOSI_GPIO_CLK | SD_SPI_MISO_GPIO_CLK |
 8002ace:	f04f 000c 	mov.w	r0, #12
 8002ad2:	f04f 0101 	mov.w	r1, #1
 8002ad6:	f001 ff7d 	bl	80049d4 <RCC_APB2PeriphClockCmd>
                         SD_SPI_SCK_GPIO_CLK, ENABLE);

  /*!< SD_SPI Periph clock enable */
  RCC_APB2PeriphClockCmd(SD_SPI_CLK, ENABLE); 
 8002ada:	f44f 5080 	mov.w	r0, #4096	; 0x1000
 8002ade:	f04f 0101 	mov.w	r1, #1
 8002ae2:	f001 ff77 	bl	80049d4 <RCC_APB2PeriphClockCmd>

  
  /*!< Configure SD_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_SCK_PIN;
 8002ae6:	f04f 0320 	mov.w	r3, #32
 8002aea:	82bb      	strh	r3, [r7, #20]
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 8002aec:	f04f 0303 	mov.w	r3, #3
 8002af0:	75bb      	strb	r3, [r7, #22]
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
 8002af2:	f04f 0318 	mov.w	r3, #24
 8002af6:	75fb      	strb	r3, [r7, #23]
  GPIO_Init(SD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
 8002af8:	f107 0314 	add.w	r3, r7, #20
 8002afc:	4826      	ldr	r0, [pc, #152]	; (8002b98 <SD_LowLevel_Init+0xd0>)
 8002afe:	4619      	mov	r1, r3
 8002b00:	f001 fcfb 	bl	80044fa <GPIO_Init>

  /*!< Configure SD_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_MOSI_PIN;
 8002b04:	f04f 0380 	mov.w	r3, #128	; 0x80
 8002b08:	82bb      	strh	r3, [r7, #20]
  GPIO_Init(SD_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
 8002b0a:	f107 0314 	add.w	r3, r7, #20
 8002b0e:	4822      	ldr	r0, [pc, #136]	; (8002b98 <SD_LowLevel_Init+0xd0>)
 8002b10:	4619      	mov	r1, r3
 8002b12:	f001 fcf2 	bl	80044fa <GPIO_Init>

  /*!< Configure SD_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_MISO_PIN;
 8002b16:	f04f 0340 	mov.w	r3, #64	; 0x40
 8002b1a:	82bb      	strh	r3, [r7, #20]
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
 8002b1c:	f04f 0304 	mov.w	r3, #4
 8002b20:	75fb      	strb	r3, [r7, #23]
  GPIO_Init(SD_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
 8002b22:	f107 0314 	add.w	r3, r7, #20
 8002b26:	481c      	ldr	r0, [pc, #112]	; (8002b98 <SD_LowLevel_Init+0xd0>)
 8002b28:	4619      	mov	r1, r3
 8002b2a:	f001 fce6 	bl	80044fa <GPIO_Init>
  
  /*!< Configure SD_SPI_CS_PIN pin: SD Card CS pin */
  GPIO_InitStructure.GPIO_Pin = SD_CS_PIN;
 8002b2e:	f44f 4380 	mov.w	r3, #16384	; 0x4000
 8002b32:	82bb      	strh	r3, [r7, #20]
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 8002b34:	f04f 0310 	mov.w	r3, #16
 8002b38:	75fb      	strb	r3, [r7, #23]
  GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStructure);
 8002b3a:	f107 0314 	add.w	r3, r7, #20
 8002b3e:	4817      	ldr	r0, [pc, #92]	; (8002b9c <SD_LowLevel_Init+0xd4>)
 8002b40:	4619      	mov	r1, r3
 8002b42:	f001 fcda 	bl	80044fa <GPIO_Init>

  /*!< SD_SPI Config */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
 8002b46:	f04f 0300 	mov.w	r3, #0
 8002b4a:	803b      	strh	r3, [r7, #0]
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
 8002b4c:	f44f 7382 	mov.w	r3, #260	; 0x104
 8002b50:	807b      	strh	r3, [r7, #2]
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
 8002b52:	f04f 0300 	mov.w	r3, #0
 8002b56:	80bb      	strh	r3, [r7, #4]
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
 8002b58:	f04f 0302 	mov.w	r3, #2
 8002b5c:	80fb      	strh	r3, [r7, #6]
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
 8002b5e:	f04f 0301 	mov.w	r3, #1
 8002b62:	813b      	strh	r3, [r7, #8]
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
 8002b64:	f44f 7300 	mov.w	r3, #512	; 0x200
 8002b68:	817b      	strh	r3, [r7, #10]
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
 8002b6a:	f04f 0308 	mov.w	r3, #8
 8002b6e:	81bb      	strh	r3, [r7, #12]
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
 8002b70:	f04f 0300 	mov.w	r3, #0
 8002b74:	81fb      	strh	r3, [r7, #14]
  SPI_InitStructure.SPI_CRCPolynomial = 7;
 8002b76:	f04f 0307 	mov.w	r3, #7
 8002b7a:	823b      	strh	r3, [r7, #16]
  SPI_Init(SD_SPI, &SPI_InitStructure);
 8002b7c:	463b      	mov	r3, r7
 8002b7e:	4808      	ldr	r0, [pc, #32]	; (8002ba0 <SD_LowLevel_Init+0xd8>)
 8002b80:	4619      	mov	r1, r3
 8002b82:	f001 ffdd 	bl	8004b40 <SPI_Init>
  
  SPI_Cmd(SD_SPI, ENABLE); /*!< SD_SPI enable */
 8002b86:	4806      	ldr	r0, [pc, #24]	; (8002ba0 <SD_LowLevel_Init+0xd8>)
 8002b88:	f04f 0101 	mov.w	r1, #1
 8002b8c:	f002 f85d 	bl	8004c4a <SPI_Cmd>
}
 8002b90:	f107 0718 	add.w	r7, r7, #24
 8002b94:	46bd      	mov	sp, r7
 8002b96:	bd80      	pop	{r7, pc}
 8002b98:	40010800 	.word	0x40010800
 8002b9c:	40010c00 	.word	0x40010c00
 8002ba0:	40013000 	.word	0x40013000

08002ba4 <SD_DeInit>:
  * @brief  DeInitializes the SD/SD communication.
  * @param  None
  * @retval None
  */
void SD_DeInit(void)
{
 8002ba4:	b580      	push	{r7, lr}
 8002ba6:	af00      	add	r7, sp, #0
  SD_LowLevel_DeInit();
 8002ba8:	f7ff ff4c 	bl	8002a44 <SD_LowLevel_DeInit>
}
 8002bac:	bd80      	pop	{r7, pc}
 8002bae:	bf00      	nop

08002bb0 <SD_Init>:
  * @retval The SD Response: 
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_Init(void)
{
 8002bb0:	b580      	push	{r7, lr}
 8002bb2:	b082      	sub	sp, #8
 8002bb4:	af00      	add	r7, sp, #0
  uint32_t i = 0;
 8002bb6:	f04f 0300 	mov.w	r3, #0
 8002bba:	607b      	str	r3, [r7, #4]

  /*!< Initialize SD_SPI */
  SD_LowLevel_Init(); 
 8002bbc:	f7ff ff84 	bl	8002ac8 <SD_LowLevel_Init>

  /*!< SD chip select high */
  SD_CS_HIGH();
 8002bc0:	480d      	ldr	r0, [pc, #52]	; (8002bf8 <SD_Init+0x48>)
 8002bc2:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002bc6:	f001 fcfe 	bl	80045c6 <GPIO_SetBits>

  /*!< Send dummy byte 0xFF, 10 times with CS high */
  /*!< Rise CS and MOSI for 80 clocks cycles */
  for (i = 0; i <= 9; i++)
 8002bca:	f04f 0300 	mov.w	r3, #0
 8002bce:	607b      	str	r3, [r7, #4]
 8002bd0:	e007      	b.n	8002be2 <SD_Init+0x32>
  {
    /*!< Send dummy byte 0xFF */
    SD_WriteByte(SD_DUMMY_BYTE);
 8002bd2:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8002bd6:	f000 fd6b 	bl	80036b0 <SD_WriteByte>
  /*!< SD chip select high */
  SD_CS_HIGH();

  /*!< Send dummy byte 0xFF, 10 times with CS high */
  /*!< Rise CS and MOSI for 80 clocks cycles */
  for (i = 0; i <= 9; i++)
 8002bda:	687b      	ldr	r3, [r7, #4]
 8002bdc:	f103 0301 	add.w	r3, r3, #1
 8002be0:	607b      	str	r3, [r7, #4]
 8002be2:	687b      	ldr	r3, [r7, #4]
 8002be4:	2b09      	cmp	r3, #9
 8002be6:	d9f4      	bls.n	8002bd2 <SD_Init+0x22>
    /*!< Send dummy byte 0xFF */
    SD_WriteByte(SD_DUMMY_BYTE);
  }
  /*------------Put SD in SPI mode--------------*/
  /*!< SD initialized and set to SPI mode properly */
  return (SD_GoIdleState());
 8002be8:	f000 fd1c 	bl	8003624 <SD_GoIdleState>
 8002bec:	4603      	mov	r3, r0
}
 8002bee:	4618      	mov	r0, r3
 8002bf0:	f107 0708 	add.w	r7, r7, #8
 8002bf4:	46bd      	mov	sp, r7
 8002bf6:	bd80      	pop	{r7, pc}
 8002bf8:	40010c00 	.word	0x40010c00

08002bfc <SD_GetCardInfo>:
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
 8002bfc:	b580      	push	{r7, lr}
 8002bfe:	b084      	sub	sp, #16
 8002c00:	af00      	add	r7, sp, #0
 8002c02:	6078      	str	r0, [r7, #4]
  SD_Error status = SD_RESPONSE_FAILURE;
 8002c04:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002c08:	73fb      	strb	r3, [r7, #15]

  status = SD_GetCSDRegister(&(cardinfo->SD_csd));
 8002c0a:	687b      	ldr	r3, [r7, #4]
 8002c0c:	4618      	mov	r0, r3
 8002c0e:	f000 f9d1 	bl	8002fb4 <SD_GetCSDRegister>
 8002c12:	4603      	mov	r3, r0
 8002c14:	73fb      	strb	r3, [r7, #15]
  status = SD_GetCIDRegister(&(cardinfo->SD_cid));
 8002c16:	687b      	ldr	r3, [r7, #4]
 8002c18:	f103 032c 	add.w	r3, r3, #44	; 0x2c
 8002c1c:	4618      	mov	r0, r3
 8002c1e:	f000 fb65 	bl	80032ec <SD_GetCIDRegister>
 8002c22:	4603      	mov	r3, r0
 8002c24:	73fb      	strb	r3, [r7, #15]
  cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
 8002c26:	687b      	ldr	r3, [r7, #4]
 8002c28:	691b      	ldr	r3, [r3, #16]
 8002c2a:	f103 0201 	add.w	r2, r3, #1
 8002c2e:	687b      	ldr	r3, [r7, #4]
 8002c30:	645a      	str	r2, [r3, #68]	; 0x44
  cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
 8002c32:	687b      	ldr	r3, [r7, #4]
 8002c34:	6c5a      	ldr	r2, [r3, #68]	; 0x44
 8002c36:	687b      	ldr	r3, [r7, #4]
 8002c38:	7e1b      	ldrb	r3, [r3, #24]
 8002c3a:	b2db      	uxtb	r3, r3
 8002c3c:	f103 0302 	add.w	r3, r3, #2
 8002c40:	fa02 f203 	lsl.w	r2, r2, r3
 8002c44:	687b      	ldr	r3, [r7, #4]
 8002c46:	645a      	str	r2, [r3, #68]	; 0x44
  cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
 8002c48:	687b      	ldr	r3, [r7, #4]
 8002c4a:	7a1b      	ldrb	r3, [r3, #8]
 8002c4c:	b2db      	uxtb	r3, r3
 8002c4e:	f04f 0201 	mov.w	r2, #1
 8002c52:	fa02 f303 	lsl.w	r3, r2, r3
 8002c56:	461a      	mov	r2, r3
 8002c58:	687b      	ldr	r3, [r7, #4]
 8002c5a:	649a      	str	r2, [r3, #72]	; 0x48
  cardinfo->CardCapacity *= cardinfo->CardBlockSize;
 8002c5c:	687b      	ldr	r3, [r7, #4]
 8002c5e:	6c5b      	ldr	r3, [r3, #68]	; 0x44
 8002c60:	687a      	ldr	r2, [r7, #4]
 8002c62:	6c92      	ldr	r2, [r2, #72]	; 0x48
 8002c64:	fb02 f203 	mul.w	r2, r2, r3
 8002c68:	687b      	ldr	r3, [r7, #4]
 8002c6a:	645a      	str	r2, [r3, #68]	; 0x44

  /*!< Returns the reponse */
  return status;
 8002c6c:	7bfb      	ldrb	r3, [r7, #15]
}
 8002c6e:	4618      	mov	r0, r3
 8002c70:	f107 0710 	add.w	r7, r7, #16
 8002c74:	46bd      	mov	sp, r7
 8002c76:	bd80      	pop	{r7, pc}

08002c78 <SD_ReadBlock>:
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_ReadBlock(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t BlockSize)
{
 8002c78:	b580      	push	{r7, lr}
 8002c7a:	b086      	sub	sp, #24
 8002c7c:	af00      	add	r7, sp, #0
 8002c7e:	60f8      	str	r0, [r7, #12]
 8002c80:	60b9      	str	r1, [r7, #8]
 8002c82:	4613      	mov	r3, r2
 8002c84:	80fb      	strh	r3, [r7, #6]
  uint32_t i = 0;
 8002c86:	f04f 0300 	mov.w	r3, #0
 8002c8a:	617b      	str	r3, [r7, #20]
  SD_Error rvalue = SD_RESPONSE_FAILURE;
 8002c8c:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002c90:	74fb      	strb	r3, [r7, #19]

  /*!< SD chip select low */
  SD_CS_LOW();
 8002c92:	4823      	ldr	r0, [pc, #140]	; (8002d20 <SD_ReadBlock+0xa8>)
 8002c94:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002c98:	f001 fc97 	bl	80045ca <GPIO_ResetBits>
  
  /*!< Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block */
  SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK, ReadAddr, 0xFF);
 8002c9c:	f04f 0011 	mov.w	r0, #17
 8002ca0:	68b9      	ldr	r1, [r7, #8]
 8002ca2:	f04f 02ff 	mov.w	r2, #255	; 0xff
 8002ca6:	f000 fbef 	bl	8003488 <SD_SendCmd>
  
  /*!< Check if the SD acknowledged the read block command: R1 response (0x00: no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
 8002caa:	f04f 0000 	mov.w	r0, #0
 8002cae:	f000 fc63 	bl	8003578 <SD_GetResponse>
 8002cb2:	4603      	mov	r3, r0
 8002cb4:	2b00      	cmp	r3, #0
 8002cb6:	d123      	bne.n	8002d00 <SD_ReadBlock+0x88>
  {
    /*!< Now look for the data token to signify the start of the data */
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
 8002cb8:	f04f 00fe 	mov.w	r0, #254	; 0xfe
 8002cbc:	f000 fc5c 	bl	8003578 <SD_GetResponse>
 8002cc0:	4603      	mov	r3, r0
 8002cc2:	2b00      	cmp	r3, #0
 8002cc4:	d11c      	bne.n	8002d00 <SD_ReadBlock+0x88>
    {
      /*!< Read the SD block data : read NumByteToRead data */
      for (i = 0; i < BlockSize; i++)
 8002cc6:	f04f 0300 	mov.w	r3, #0
 8002cca:	617b      	str	r3, [r7, #20]
 8002ccc:	e00d      	b.n	8002cea <SD_ReadBlock+0x72>
      {
        /*!< Save the received data */
        *pBuffer = SD_ReadByte();
 8002cce:	f000 fd19 	bl	8003704 <SD_ReadByte>
 8002cd2:	4603      	mov	r3, r0
 8002cd4:	461a      	mov	r2, r3
 8002cd6:	68fb      	ldr	r3, [r7, #12]
 8002cd8:	701a      	strb	r2, [r3, #0]
       
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
 8002cda:	68fb      	ldr	r3, [r7, #12]
 8002cdc:	f103 0301 	add.w	r3, r3, #1
 8002ce0:	60fb      	str	r3, [r7, #12]
  {
    /*!< Now look for the data token to signify the start of the data */
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      /*!< Read the SD block data : read NumByteToRead data */
      for (i = 0; i < BlockSize; i++)
 8002ce2:	697b      	ldr	r3, [r7, #20]
 8002ce4:	f103 0301 	add.w	r3, r3, #1
 8002ce8:	617b      	str	r3, [r7, #20]
 8002cea:	88fa      	ldrh	r2, [r7, #6]
 8002cec:	697b      	ldr	r3, [r7, #20]
 8002cee:	429a      	cmp	r2, r3
 8002cf0:	d8ed      	bhi.n	8002cce <SD_ReadBlock+0x56>
       
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
      }
      /*!< Get CRC bytes (not really needed by us, but required by SD) */
      SD_ReadByte();
 8002cf2:	f000 fd07 	bl	8003704 <SD_ReadByte>
      SD_ReadByte();
 8002cf6:	f000 fd05 	bl	8003704 <SD_ReadByte>
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
 8002cfa:	f04f 0300 	mov.w	r3, #0
 8002cfe:	74fb      	strb	r3, [r7, #19]
    }
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
 8002d00:	4807      	ldr	r0, [pc, #28]	; (8002d20 <SD_ReadBlock+0xa8>)
 8002d02:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002d06:	f001 fc5e 	bl	80045c6 <GPIO_SetBits>
  
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
 8002d0a:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8002d0e:	f000 fccf 	bl	80036b0 <SD_WriteByte>
  
  /*!< Returns the reponse */
  return rvalue;
 8002d12:	7cfb      	ldrb	r3, [r7, #19]
}
 8002d14:	4618      	mov	r0, r3
 8002d16:	f107 0718 	add.w	r7, r7, #24
 8002d1a:	46bd      	mov	sp, r7
 8002d1c:	bd80      	pop	{r7, pc}
 8002d1e:	bf00      	nop
 8002d20:	40010c00 	.word	0x40010c00

08002d24 <SD_ReadMultiBlocks>:
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_ReadMultiBlocks(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
 8002d24:	b580      	push	{r7, lr}
 8002d26:	b088      	sub	sp, #32
 8002d28:	af00      	add	r7, sp, #0
 8002d2a:	60f8      	str	r0, [r7, #12]
 8002d2c:	60b9      	str	r1, [r7, #8]
 8002d2e:	603b      	str	r3, [r7, #0]
 8002d30:	4613      	mov	r3, r2
 8002d32:	80fb      	strh	r3, [r7, #6]
  uint32_t i = 0, Offset = 0;
 8002d34:	f04f 0300 	mov.w	r3, #0
 8002d38:	61fb      	str	r3, [r7, #28]
 8002d3a:	f04f 0300 	mov.w	r3, #0
 8002d3e:	61bb      	str	r3, [r7, #24]
  SD_Error rvalue = SD_RESPONSE_FAILURE;
 8002d40:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002d44:	75fb      	strb	r3, [r7, #23]
  
  /*!< SD chip select low */
  SD_CS_LOW();
 8002d46:	4830      	ldr	r0, [pc, #192]	; (8002e08 <SD_ReadMultiBlocks+0xe4>)
 8002d48:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002d4c:	f001 fc3d 	bl	80045ca <GPIO_ResetBits>
  /*!< Data transfer */
  while (NumberOfBlocks--)
 8002d50:	e03f      	b.n	8002dd2 <SD_ReadMultiBlocks+0xae>
  {
    /*!< Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block */
    SD_SendCmd (SD_CMD_READ_SINGLE_BLOCK, ReadAddr + Offset, 0xFF);
 8002d52:	68ba      	ldr	r2, [r7, #8]
 8002d54:	69bb      	ldr	r3, [r7, #24]
 8002d56:	18d3      	adds	r3, r2, r3
 8002d58:	f04f 0011 	mov.w	r0, #17
 8002d5c:	4619      	mov	r1, r3
 8002d5e:	f04f 02ff 	mov.w	r2, #255	; 0xff
 8002d62:	f000 fb91 	bl	8003488 <SD_SendCmd>
    /*!< Check if the SD acknowledged the read block command: R1 response (0x00: no errors) */
    if (SD_GetResponse(SD_RESPONSE_NO_ERROR))
 8002d66:	f04f 0000 	mov.w	r0, #0
 8002d6a:	f000 fc05 	bl	8003578 <SD_GetResponse>
 8002d6e:	4603      	mov	r3, r0
 8002d70:	2b00      	cmp	r3, #0
 8002d72:	d002      	beq.n	8002d7a <SD_ReadMultiBlocks+0x56>
    {
      return  SD_RESPONSE_FAILURE;
 8002d74:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002d78:	e041      	b.n	8002dfe <SD_ReadMultiBlocks+0xda>
    }
    /*!< Now look for the data token to signify the start of the data */
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
 8002d7a:	f04f 00fe 	mov.w	r0, #254	; 0xfe
 8002d7e:	f000 fbfb 	bl	8003578 <SD_GetResponse>
 8002d82:	4603      	mov	r3, r0
 8002d84:	2b00      	cmp	r3, #0
 8002d86:	d121      	bne.n	8002dcc <SD_ReadMultiBlocks+0xa8>
    {
      /*!< Read the SD block data : read NumByteToRead data */
      for (i = 0; i < BlockSize; i++)
 8002d88:	f04f 0300 	mov.w	r3, #0
 8002d8c:	61fb      	str	r3, [r7, #28]
 8002d8e:	e00d      	b.n	8002dac <SD_ReadMultiBlocks+0x88>
      {
        /*!< Read the pointed data */
        *pBuffer = SD_ReadByte();
 8002d90:	f000 fcb8 	bl	8003704 <SD_ReadByte>
 8002d94:	4603      	mov	r3, r0
 8002d96:	461a      	mov	r2, r3
 8002d98:	68fb      	ldr	r3, [r7, #12]
 8002d9a:	701a      	strb	r2, [r3, #0]
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
 8002d9c:	68fb      	ldr	r3, [r7, #12]
 8002d9e:	f103 0301 	add.w	r3, r3, #1
 8002da2:	60fb      	str	r3, [r7, #12]
    }
    /*!< Now look for the data token to signify the start of the data */
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      /*!< Read the SD block data : read NumByteToRead data */
      for (i = 0; i < BlockSize; i++)
 8002da4:	69fb      	ldr	r3, [r7, #28]
 8002da6:	f103 0301 	add.w	r3, r3, #1
 8002daa:	61fb      	str	r3, [r7, #28]
 8002dac:	88fa      	ldrh	r2, [r7, #6]
 8002dae:	69fb      	ldr	r3, [r7, #28]
 8002db0:	429a      	cmp	r2, r3
 8002db2:	d8ed      	bhi.n	8002d90 <SD_ReadMultiBlocks+0x6c>
        *pBuffer = SD_ReadByte();
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
      }
      /*!< Set next read address*/
      Offset += 512;
 8002db4:	69bb      	ldr	r3, [r7, #24]
 8002db6:	f503 7300 	add.w	r3, r3, #512	; 0x200
 8002dba:	61bb      	str	r3, [r7, #24]
      /*!< get CRC bytes (not really needed by us, but required by SD) */
      SD_ReadByte();
 8002dbc:	f000 fca2 	bl	8003704 <SD_ReadByte>
      SD_ReadByte();
 8002dc0:	f000 fca0 	bl	8003704 <SD_ReadByte>
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
 8002dc4:	f04f 0300 	mov.w	r3, #0
 8002dc8:	75fb      	strb	r3, [r7, #23]
 8002dca:	e002      	b.n	8002dd2 <SD_ReadMultiBlocks+0xae>
    }
    else
    {
      /*!< Set response value to failure */
      rvalue = SD_RESPONSE_FAILURE;
 8002dcc:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002dd0:	75fb      	strb	r3, [r7, #23]
  SD_Error rvalue = SD_RESPONSE_FAILURE;
  
  /*!< SD chip select low */
  SD_CS_LOW();
  /*!< Data transfer */
  while (NumberOfBlocks--)
 8002dd2:	683b      	ldr	r3, [r7, #0]
 8002dd4:	2b00      	cmp	r3, #0
 8002dd6:	bf0c      	ite	eq
 8002dd8:	2300      	moveq	r3, #0
 8002dda:	2301      	movne	r3, #1
 8002ddc:	b2db      	uxtb	r3, r3
 8002dde:	683a      	ldr	r2, [r7, #0]
 8002de0:	f102 32ff 	add.w	r2, r2, #4294967295
 8002de4:	603a      	str	r2, [r7, #0]
 8002de6:	2b00      	cmp	r3, #0
 8002de8:	d1b3      	bne.n	8002d52 <SD_ReadMultiBlocks+0x2e>
      /*!< Set response value to failure */
      rvalue = SD_RESPONSE_FAILURE;
    }
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
 8002dea:	4807      	ldr	r0, [pc, #28]	; (8002e08 <SD_ReadMultiBlocks+0xe4>)
 8002dec:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002df0:	f001 fbe9 	bl	80045c6 <GPIO_SetBits>
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
 8002df4:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8002df8:	f000 fc5a 	bl	80036b0 <SD_WriteByte>
  /*!< Returns the reponse */
  return rvalue;
 8002dfc:	7dfb      	ldrb	r3, [r7, #23]
}
 8002dfe:	4618      	mov	r0, r3
 8002e00:	f107 0720 	add.w	r7, r7, #32
 8002e04:	46bd      	mov	sp, r7
 8002e06:	bd80      	pop	{r7, pc}
 8002e08:	40010c00 	.word	0x40010c00

08002e0c <SD_WriteBlock>:
  * @retval The SD Response: 
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_WriteBlock(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t BlockSize)
{
 8002e0c:	b580      	push	{r7, lr}
 8002e0e:	b086      	sub	sp, #24
 8002e10:	af00      	add	r7, sp, #0
 8002e12:	60f8      	str	r0, [r7, #12]
 8002e14:	60b9      	str	r1, [r7, #8]
 8002e16:	4613      	mov	r3, r2
 8002e18:	80fb      	strh	r3, [r7, #6]
  uint32_t i = 0;
 8002e1a:	f04f 0300 	mov.w	r3, #0
 8002e1e:	617b      	str	r3, [r7, #20]
  SD_Error rvalue = SD_RESPONSE_FAILURE;
 8002e20:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002e24:	74fb      	strb	r3, [r7, #19]

  /*!< SD chip select low */
  SD_CS_LOW();
 8002e26:	4825      	ldr	r0, [pc, #148]	; (8002ebc <SD_WriteBlock+0xb0>)
 8002e28:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002e2c:	f001 fbcd 	bl	80045ca <GPIO_ResetBits>

  /*!< Send CMD24 (SD_CMD_WRITE_SINGLE_BLOCK) to write multiple block */
  SD_SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, WriteAddr, 0xFF);
 8002e30:	f04f 0018 	mov.w	r0, #24
 8002e34:	68b9      	ldr	r1, [r7, #8]
 8002e36:	f04f 02ff 	mov.w	r2, #255	; 0xff
 8002e3a:	f000 fb25 	bl	8003488 <SD_SendCmd>
  
  /*!< Check if the SD acknowledged the write block command: R1 response (0x00: no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
 8002e3e:	f04f 0000 	mov.w	r0, #0
 8002e42:	f000 fb99 	bl	8003578 <SD_GetResponse>
 8002e46:	4603      	mov	r3, r0
 8002e48:	2b00      	cmp	r3, #0
 8002e4a:	d128      	bne.n	8002e9e <SD_WriteBlock+0x92>
  {
    /*!< Send a dummy byte */
    SD_WriteByte(SD_DUMMY_BYTE);
 8002e4c:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8002e50:	f000 fc2e 	bl	80036b0 <SD_WriteByte>

    /*!< Send the data token to signify the start of the data */
    SD_WriteByte(0xFE);
 8002e54:	f04f 00fe 	mov.w	r0, #254	; 0xfe
 8002e58:	f000 fc2a 	bl	80036b0 <SD_WriteByte>

    /*!< Write the block data to SD : write count data by block */
    for (i = 0; i < BlockSize; i++)
 8002e5c:	f04f 0300 	mov.w	r3, #0
 8002e60:	617b      	str	r3, [r7, #20]
 8002e62:	e00c      	b.n	8002e7e <SD_WriteBlock+0x72>
    {
      /*!< Send the pointed byte */
      SD_WriteByte(*pBuffer);
 8002e64:	68fb      	ldr	r3, [r7, #12]
 8002e66:	781b      	ldrb	r3, [r3, #0]
 8002e68:	4618      	mov	r0, r3
 8002e6a:	f000 fc21 	bl	80036b0 <SD_WriteByte>
      /*!< Point to the next location where the byte read will be saved */
      pBuffer++;
 8002e6e:	68fb      	ldr	r3, [r7, #12]
 8002e70:	f103 0301 	add.w	r3, r3, #1
 8002e74:	60fb      	str	r3, [r7, #12]

    /*!< Send the data token to signify the start of the data */
    SD_WriteByte(0xFE);

    /*!< Write the block data to SD : write count data by block */
    for (i = 0; i < BlockSize; i++)
 8002e76:	697b      	ldr	r3, [r7, #20]
 8002e78:	f103 0301 	add.w	r3, r3, #1
 8002e7c:	617b      	str	r3, [r7, #20]
 8002e7e:	88fa      	ldrh	r2, [r7, #6]
 8002e80:	697b      	ldr	r3, [r7, #20]
 8002e82:	429a      	cmp	r2, r3
 8002e84:	d8ee      	bhi.n	8002e64 <SD_WriteBlock+0x58>
      SD_WriteByte(*pBuffer);
      /*!< Point to the next location where the byte read will be saved */
      pBuffer++;
    }
    /*!< Put CRC bytes (not really needed by us, but required by SD) */
    SD_ReadByte();
 8002e86:	f000 fc3d 	bl	8003704 <SD_ReadByte>
    SD_ReadByte();
 8002e8a:	f000 fc3b 	bl	8003704 <SD_ReadByte>

    /*!< Read data response */
    if (SD_GetDataResponse() == SD_DATA_OK)
 8002e8e:	f000 fb37 	bl	8003500 <SD_GetDataResponse>
 8002e92:	4603      	mov	r3, r0
 8002e94:	2b05      	cmp	r3, #5
 8002e96:	d102      	bne.n	8002e9e <SD_WriteBlock+0x92>
    {
      rvalue = SD_RESPONSE_NO_ERROR;
 8002e98:	f04f 0300 	mov.w	r3, #0
 8002e9c:	74fb      	strb	r3, [r7, #19]
    }
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
 8002e9e:	4807      	ldr	r0, [pc, #28]	; (8002ebc <SD_WriteBlock+0xb0>)
 8002ea0:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002ea4:	f001 fb8f 	bl	80045c6 <GPIO_SetBits>
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
 8002ea8:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8002eac:	f000 fc00 	bl	80036b0 <SD_WriteByte>

  /*!< Returns the reponse */
  return rvalue;
 8002eb0:	7cfb      	ldrb	r3, [r7, #19]
}
 8002eb2:	4618      	mov	r0, r3
 8002eb4:	f107 0718 	add.w	r7, r7, #24
 8002eb8:	46bd      	mov	sp, r7
 8002eba:	bd80      	pop	{r7, pc}
 8002ebc:	40010c00 	.word	0x40010c00

08002ec0 <SD_WriteMultiBlocks>:
  * @retval The SD Response: 
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_WriteMultiBlocks(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
 8002ec0:	b580      	push	{r7, lr}
 8002ec2:	b088      	sub	sp, #32
 8002ec4:	af00      	add	r7, sp, #0
 8002ec6:	60f8      	str	r0, [r7, #12]
 8002ec8:	60b9      	str	r1, [r7, #8]
 8002eca:	603b      	str	r3, [r7, #0]
 8002ecc:	4613      	mov	r3, r2
 8002ece:	80fb      	strh	r3, [r7, #6]
  uint32_t i = 0, Offset = 0;
 8002ed0:	f04f 0300 	mov.w	r3, #0
 8002ed4:	61fb      	str	r3, [r7, #28]
 8002ed6:	f04f 0300 	mov.w	r3, #0
 8002eda:	61bb      	str	r3, [r7, #24]
  SD_Error rvalue = SD_RESPONSE_FAILURE;
 8002edc:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002ee0:	75fb      	strb	r3, [r7, #23]

  /*!< SD chip select low */
  SD_CS_LOW();
 8002ee2:	4833      	ldr	r0, [pc, #204]	; (8002fb0 <SD_WriteMultiBlocks+0xf0>)
 8002ee4:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002ee8:	f001 fb6f 	bl	80045ca <GPIO_ResetBits>
  /*!< Data transfer */
  while (NumberOfBlocks--)
 8002eec:	e044      	b.n	8002f78 <SD_WriteMultiBlocks+0xb8>
  {
    /*!< Send CMD24 (SD_CMD_WRITE_SINGLE_BLOCK) to write blocks */
    SD_SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, WriteAddr + Offset, 0xFF);
 8002eee:	68ba      	ldr	r2, [r7, #8]
 8002ef0:	69bb      	ldr	r3, [r7, #24]
 8002ef2:	18d3      	adds	r3, r2, r3
 8002ef4:	f04f 0018 	mov.w	r0, #24
 8002ef8:	4619      	mov	r1, r3
 8002efa:	f04f 02ff 	mov.w	r2, #255	; 0xff
 8002efe:	f000 fac3 	bl	8003488 <SD_SendCmd>
    /*!< Check if the SD acknowledged the write block command: R1 response (0x00: no errors) */
    if (SD_GetResponse(SD_RESPONSE_NO_ERROR))
 8002f02:	f04f 0000 	mov.w	r0, #0
 8002f06:	f000 fb37 	bl	8003578 <SD_GetResponse>
 8002f0a:	4603      	mov	r3, r0
 8002f0c:	2b00      	cmp	r3, #0
 8002f0e:	d002      	beq.n	8002f16 <SD_WriteMultiBlocks+0x56>
    {
      return SD_RESPONSE_FAILURE;
 8002f10:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002f14:	e046      	b.n	8002fa4 <SD_WriteMultiBlocks+0xe4>
    }
    /*!< Send dummy byte */
    SD_WriteByte(SD_DUMMY_BYTE);
 8002f16:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8002f1a:	f000 fbc9 	bl	80036b0 <SD_WriteByte>
    /*!< Send the data token to signify the start of the data */
    SD_WriteByte(SD_START_DATA_SINGLE_BLOCK_WRITE);
 8002f1e:	f04f 00fe 	mov.w	r0, #254	; 0xfe
 8002f22:	f000 fbc5 	bl	80036b0 <SD_WriteByte>
    /*!< Write the block data to SD : write count data by block */
    for (i = 0; i < BlockSize; i++)
 8002f26:	f04f 0300 	mov.w	r3, #0
 8002f2a:	61fb      	str	r3, [r7, #28]
 8002f2c:	e00c      	b.n	8002f48 <SD_WriteMultiBlocks+0x88>
    {
      /*!< Send the pointed byte */
      SD_WriteByte(*pBuffer);
 8002f2e:	68fb      	ldr	r3, [r7, #12]
 8002f30:	781b      	ldrb	r3, [r3, #0]
 8002f32:	4618      	mov	r0, r3
 8002f34:	f000 fbbc 	bl	80036b0 <SD_WriteByte>
      /*!< Point to the next location where the byte read will be saved */
      pBuffer++;
 8002f38:	68fb      	ldr	r3, [r7, #12]
 8002f3a:	f103 0301 	add.w	r3, r3, #1
 8002f3e:	60fb      	str	r3, [r7, #12]
    /*!< Send dummy byte */
    SD_WriteByte(SD_DUMMY_BYTE);
    /*!< Send the data token to signify the start of the data */
    SD_WriteByte(SD_START_DATA_SINGLE_BLOCK_WRITE);
    /*!< Write the block data to SD : write count data by block */
    for (i = 0; i < BlockSize; i++)
 8002f40:	69fb      	ldr	r3, [r7, #28]
 8002f42:	f103 0301 	add.w	r3, r3, #1
 8002f46:	61fb      	str	r3, [r7, #28]
 8002f48:	88fa      	ldrh	r2, [r7, #6]
 8002f4a:	69fb      	ldr	r3, [r7, #28]
 8002f4c:	429a      	cmp	r2, r3
 8002f4e:	d8ee      	bhi.n	8002f2e <SD_WriteMultiBlocks+0x6e>
      SD_WriteByte(*pBuffer);
      /*!< Point to the next location where the byte read will be saved */
      pBuffer++;
    }
    /*!< Set next write address */
    Offset += 512;
 8002f50:	69bb      	ldr	r3, [r7, #24]
 8002f52:	f503 7300 	add.w	r3, r3, #512	; 0x200
 8002f56:	61bb      	str	r3, [r7, #24]
    /*!< Put CRC bytes (not really needed by us, but required by SD) */
    SD_ReadByte();
 8002f58:	f000 fbd4 	bl	8003704 <SD_ReadByte>
    SD_ReadByte();
 8002f5c:	f000 fbd2 	bl	8003704 <SD_ReadByte>
    /*!< Read data response */
    if (SD_GetDataResponse() == SD_DATA_OK)
 8002f60:	f000 face 	bl	8003500 <SD_GetDataResponse>
 8002f64:	4603      	mov	r3, r0
 8002f66:	2b05      	cmp	r3, #5
 8002f68:	d103      	bne.n	8002f72 <SD_WriteMultiBlocks+0xb2>
    {
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
 8002f6a:	f04f 0300 	mov.w	r3, #0
 8002f6e:	75fb      	strb	r3, [r7, #23]
 8002f70:	e002      	b.n	8002f78 <SD_WriteMultiBlocks+0xb8>
    }
    else
    {
      /*!< Set response value to failure */
      rvalue = SD_RESPONSE_FAILURE;
 8002f72:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002f76:	75fb      	strb	r3, [r7, #23]
  SD_Error rvalue = SD_RESPONSE_FAILURE;

  /*!< SD chip select low */
  SD_CS_LOW();
  /*!< Data transfer */
  while (NumberOfBlocks--)
 8002f78:	683b      	ldr	r3, [r7, #0]
 8002f7a:	2b00      	cmp	r3, #0
 8002f7c:	bf0c      	ite	eq
 8002f7e:	2300      	moveq	r3, #0
 8002f80:	2301      	movne	r3, #1
 8002f82:	b2db      	uxtb	r3, r3
 8002f84:	683a      	ldr	r2, [r7, #0]
 8002f86:	f102 32ff 	add.w	r2, r2, #4294967295
 8002f8a:	603a      	str	r2, [r7, #0]
 8002f8c:	2b00      	cmp	r3, #0
 8002f8e:	d1ae      	bne.n	8002eee <SD_WriteMultiBlocks+0x2e>
      /*!< Set response value to failure */
      rvalue = SD_RESPONSE_FAILURE;
    }
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
 8002f90:	4807      	ldr	r0, [pc, #28]	; (8002fb0 <SD_WriteMultiBlocks+0xf0>)
 8002f92:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002f96:	f001 fb16 	bl	80045c6 <GPIO_SetBits>
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
 8002f9a:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8002f9e:	f000 fb87 	bl	80036b0 <SD_WriteByte>
  /*!< Returns the reponse */
  return rvalue;
 8002fa2:	7dfb      	ldrb	r3, [r7, #23]
}
 8002fa4:	4618      	mov	r0, r3
 8002fa6:	f107 0720 	add.w	r7, r7, #32
 8002faa:	46bd      	mov	sp, r7
 8002fac:	bd80      	pop	{r7, pc}
 8002fae:	bf00      	nop
 8002fb0:	40010c00 	.word	0x40010c00

08002fb4 <SD_GetCSDRegister>:
  * @retval The SD Response: 
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GetCSDRegister(SD_CSD* SD_csd)
{
 8002fb4:	b580      	push	{r7, lr}
 8002fb6:	b088      	sub	sp, #32
 8002fb8:	af00      	add	r7, sp, #0
 8002fba:	6078      	str	r0, [r7, #4]
  uint32_t i = 0;
 8002fbc:	f04f 0300 	mov.w	r3, #0
 8002fc0:	61fb      	str	r3, [r7, #28]
  SD_Error rvalue = SD_RESPONSE_FAILURE;
 8002fc2:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8002fc6:	76fb      	strb	r3, [r7, #27]
  uint8_t CSD_Tab[16];

  /*!< SD chip select low */
  SD_CS_LOW();
 8002fc8:	480f      	ldr	r0, [pc, #60]	; (8003008 <SD_GetCSDRegister+0x54>)
 8002fca:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8002fce:	f001 fafc 	bl	80045ca <GPIO_ResetBits>
  /*!< Send CMD9 (CSD register) or CMD10(CSD register) */
  SD_SendCmd(SD_CMD_SEND_CSD, 0, 0xFF);
 8002fd2:	f04f 0009 	mov.w	r0, #9
 8002fd6:	f04f 0100 	mov.w	r1, #0
 8002fda:	f04f 02ff 	mov.w	r2, #255	; 0xff
 8002fde:	f000 fa53 	bl	8003488 <SD_SendCmd>
  /*!< Wait for response in the R1 format (0x00 is no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
 8002fe2:	f04f 0000 	mov.w	r0, #0
 8002fe6:	f000 fac7 	bl	8003578 <SD_GetResponse>
 8002fea:	4603      	mov	r3, r0
 8002fec:	2b00      	cmp	r3, #0
 8002fee:	d128      	bne.n	8003042 <SD_GetCSDRegister+0x8e>
  {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
 8002ff0:	f04f 00fe 	mov.w	r0, #254	; 0xfe
 8002ff4:	f000 fac0 	bl	8003578 <SD_GetResponse>
 8002ff8:	4603      	mov	r3, r0
 8002ffa:	2b00      	cmp	r3, #0
 8002ffc:	d116      	bne.n	800302c <SD_GetCSDRegister+0x78>
    {
      for (i = 0; i < 16; i++)
 8002ffe:	f04f 0300 	mov.w	r3, #0
 8003002:	61fb      	str	r3, [r7, #28]
 8003004:	e00f      	b.n	8003026 <SD_GetCSDRegister+0x72>
 8003006:	bf00      	nop
 8003008:	40010c00 	.word	0x40010c00
      {
        /*!< Store CSD register value on CSD_Tab */
        CSD_Tab[i] = SD_ReadByte();
 800300c:	f000 fb7a 	bl	8003704 <SD_ReadByte>
 8003010:	4603      	mov	r3, r0
 8003012:	461a      	mov	r2, r3
 8003014:	f107 0108 	add.w	r1, r7, #8
 8003018:	69fb      	ldr	r3, [r7, #28]
 800301a:	18cb      	adds	r3, r1, r3
 800301c:	701a      	strb	r2, [r3, #0]
  /*!< Wait for response in the R1 format (0x00 is no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
  {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      for (i = 0; i < 16; i++)
 800301e:	69fb      	ldr	r3, [r7, #28]
 8003020:	f103 0301 	add.w	r3, r3, #1
 8003024:	61fb      	str	r3, [r7, #28]
 8003026:	69fb      	ldr	r3, [r7, #28]
 8003028:	2b0f      	cmp	r3, #15
 800302a:	d9ef      	bls.n	800300c <SD_GetCSDRegister+0x58>
        /*!< Store CSD register value on CSD_Tab */
        CSD_Tab[i] = SD_ReadByte();
      }
    }
    /*!< Get CRC bytes (not really needed by us, but required by SD) */
    SD_WriteByte(SD_DUMMY_BYTE);
 800302c:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8003030:	f000 fb3e 	bl	80036b0 <SD_WriteByte>
    SD_WriteByte(SD_DUMMY_BYTE);
 8003034:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8003038:	f000 fb3a 	bl	80036b0 <SD_WriteByte>
    /*!< Set response value to success */
    rvalue = SD_RESPONSE_NO_ERROR;
 800303c:	f04f 0300 	mov.w	r3, #0
 8003040:	76fb      	strb	r3, [r7, #27]
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
 8003042:	488e      	ldr	r0, [pc, #568]	; (800327c <SD_GetCSDRegister+0x2c8>)
 8003044:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8003048:	f001 fabd 	bl	80045c6 <GPIO_SetBits>
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
 800304c:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8003050:	f000 fb2e 	bl	80036b0 <SD_WriteByte>

  /*!< Byte 0 */
  SD_csd->CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
 8003054:	7a3b      	ldrb	r3, [r7, #8]
 8003056:	ea4f 1393 	mov.w	r3, r3, lsr #6
 800305a:	b2da      	uxtb	r2, r3
 800305c:	687b      	ldr	r3, [r7, #4]
 800305e:	701a      	strb	r2, [r3, #0]
  SD_csd->SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
 8003060:	7a3b      	ldrb	r3, [r7, #8]
 8003062:	f003 033c 	and.w	r3, r3, #60	; 0x3c
 8003066:	ea4f 03a3 	mov.w	r3, r3, asr #2
 800306a:	b2da      	uxtb	r2, r3
 800306c:	687b      	ldr	r3, [r7, #4]
 800306e:	705a      	strb	r2, [r3, #1]
  SD_csd->Reserved1 = CSD_Tab[0] & 0x03;
 8003070:	7a3b      	ldrb	r3, [r7, #8]
 8003072:	f003 0303 	and.w	r3, r3, #3
 8003076:	b2da      	uxtb	r2, r3
 8003078:	687b      	ldr	r3, [r7, #4]
 800307a:	709a      	strb	r2, [r3, #2]

  /*!< Byte 1 */
  SD_csd->TAAC = CSD_Tab[1];
 800307c:	7a7a      	ldrb	r2, [r7, #9]
 800307e:	687b      	ldr	r3, [r7, #4]
 8003080:	70da      	strb	r2, [r3, #3]

  /*!< Byte 2 */
  SD_csd->NSAC = CSD_Tab[2];
 8003082:	7aba      	ldrb	r2, [r7, #10]
 8003084:	687b      	ldr	r3, [r7, #4]
 8003086:	711a      	strb	r2, [r3, #4]

  /*!< Byte 3 */
  SD_csd->MaxBusClkFrec = CSD_Tab[3];
 8003088:	7afa      	ldrb	r2, [r7, #11]
 800308a:	687b      	ldr	r3, [r7, #4]
 800308c:	715a      	strb	r2, [r3, #5]

  /*!< Byte 4 */
  SD_csd->CardComdClasses = CSD_Tab[4] << 4;
 800308e:	7b3b      	ldrb	r3, [r7, #12]
 8003090:	ea4f 1303 	mov.w	r3, r3, lsl #4
 8003094:	b29a      	uxth	r2, r3
 8003096:	687b      	ldr	r3, [r7, #4]
 8003098:	80da      	strh	r2, [r3, #6]

  /*!< Byte 5 */
  SD_csd->CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
 800309a:	687b      	ldr	r3, [r7, #4]
 800309c:	88db      	ldrh	r3, [r3, #6]
 800309e:	b29a      	uxth	r2, r3
 80030a0:	7b7b      	ldrb	r3, [r7, #13]
 80030a2:	ea4f 1313 	mov.w	r3, r3, lsr #4
 80030a6:	b2db      	uxtb	r3, r3
 80030a8:	ea42 0303 	orr.w	r3, r2, r3
 80030ac:	b29a      	uxth	r2, r3
 80030ae:	687b      	ldr	r3, [r7, #4]
 80030b0:	80da      	strh	r2, [r3, #6]
  SD_csd->RdBlockLen = CSD_Tab[5] & 0x0F;
 80030b2:	7b7b      	ldrb	r3, [r7, #13]
 80030b4:	f003 030f 	and.w	r3, r3, #15
 80030b8:	b2da      	uxtb	r2, r3
 80030ba:	687b      	ldr	r3, [r7, #4]
 80030bc:	721a      	strb	r2, [r3, #8]

  /*!< Byte 6 */
  SD_csd->PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
 80030be:	7bbb      	ldrb	r3, [r7, #14]
 80030c0:	ea4f 13d3 	mov.w	r3, r3, lsr #7
 80030c4:	b2da      	uxtb	r2, r3
 80030c6:	687b      	ldr	r3, [r7, #4]
 80030c8:	725a      	strb	r2, [r3, #9]
  SD_csd->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
 80030ca:	7bbb      	ldrb	r3, [r7, #14]
 80030cc:	f003 0340 	and.w	r3, r3, #64	; 0x40
 80030d0:	ea4f 13a3 	mov.w	r3, r3, asr #6
 80030d4:	b2da      	uxtb	r2, r3
 80030d6:	687b      	ldr	r3, [r7, #4]
 80030d8:	729a      	strb	r2, [r3, #10]
  SD_csd->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
 80030da:	7bbb      	ldrb	r3, [r7, #14]
 80030dc:	f003 0320 	and.w	r3, r3, #32
 80030e0:	ea4f 1363 	mov.w	r3, r3, asr #5
 80030e4:	b2da      	uxtb	r2, r3
 80030e6:	687b      	ldr	r3, [r7, #4]
 80030e8:	72da      	strb	r2, [r3, #11]
  SD_csd->DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
 80030ea:	7bbb      	ldrb	r3, [r7, #14]
 80030ec:	f003 0310 	and.w	r3, r3, #16
 80030f0:	ea4f 1323 	mov.w	r3, r3, asr #4
 80030f4:	b2da      	uxtb	r2, r3
 80030f6:	687b      	ldr	r3, [r7, #4]
 80030f8:	731a      	strb	r2, [r3, #12]
  SD_csd->Reserved2 = 0; /*!< Reserved */
 80030fa:	687b      	ldr	r3, [r7, #4]
 80030fc:	f04f 0200 	mov.w	r2, #0
 8003100:	735a      	strb	r2, [r3, #13]

  SD_csd->DeviceSize = (CSD_Tab[6] & 0x03) << 10;
 8003102:	7bbb      	ldrb	r3, [r7, #14]
 8003104:	f003 0303 	and.w	r3, r3, #3
 8003108:	ea4f 2383 	mov.w	r3, r3, lsl #10
 800310c:	461a      	mov	r2, r3
 800310e:	687b      	ldr	r3, [r7, #4]
 8003110:	611a      	str	r2, [r3, #16]

  /*!< Byte 7 */
  SD_csd->DeviceSize |= (CSD_Tab[7]) << 2;
 8003112:	687b      	ldr	r3, [r7, #4]
 8003114:	691a      	ldr	r2, [r3, #16]
 8003116:	7bfb      	ldrb	r3, [r7, #15]
 8003118:	ea4f 0383 	mov.w	r3, r3, lsl #2
 800311c:	431a      	orrs	r2, r3
 800311e:	687b      	ldr	r3, [r7, #4]
 8003120:	611a      	str	r2, [r3, #16]

  /*!< Byte 8 */
  SD_csd->DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;
 8003122:	687b      	ldr	r3, [r7, #4]
 8003124:	691a      	ldr	r2, [r3, #16]
 8003126:	7c3b      	ldrb	r3, [r7, #16]
 8003128:	ea4f 1393 	mov.w	r3, r3, lsr #6
 800312c:	b2db      	uxtb	r3, r3
 800312e:	431a      	orrs	r2, r3
 8003130:	687b      	ldr	r3, [r7, #4]
 8003132:	611a      	str	r2, [r3, #16]

  SD_csd->MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
 8003134:	7c3b      	ldrb	r3, [r7, #16]
 8003136:	f003 0338 	and.w	r3, r3, #56	; 0x38
 800313a:	ea4f 03e3 	mov.w	r3, r3, asr #3
 800313e:	b2da      	uxtb	r2, r3
 8003140:	687b      	ldr	r3, [r7, #4]
 8003142:	751a      	strb	r2, [r3, #20]
  SD_csd->MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
 8003144:	7c3b      	ldrb	r3, [r7, #16]
 8003146:	f003 0307 	and.w	r3, r3, #7
 800314a:	b2da      	uxtb	r2, r3
 800314c:	687b      	ldr	r3, [r7, #4]
 800314e:	755a      	strb	r2, [r3, #21]

  /*!< Byte 9 */
  SD_csd->MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
 8003150:	7c7b      	ldrb	r3, [r7, #17]
 8003152:	ea4f 1353 	mov.w	r3, r3, lsr #5
 8003156:	b2da      	uxtb	r2, r3
 8003158:	687b      	ldr	r3, [r7, #4]
 800315a:	759a      	strb	r2, [r3, #22]
  SD_csd->MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
 800315c:	7c7b      	ldrb	r3, [r7, #17]
 800315e:	f003 031c 	and.w	r3, r3, #28
 8003162:	ea4f 03a3 	mov.w	r3, r3, asr #2
 8003166:	b2da      	uxtb	r2, r3
 8003168:	687b      	ldr	r3, [r7, #4]
 800316a:	75da      	strb	r2, [r3, #23]
  SD_csd->DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
 800316c:	7c7b      	ldrb	r3, [r7, #17]
 800316e:	f003 0303 	and.w	r3, r3, #3
 8003172:	b2db      	uxtb	r3, r3
 8003174:	ea4f 0343 	mov.w	r3, r3, lsl #1
 8003178:	b2da      	uxtb	r2, r3
 800317a:	687b      	ldr	r3, [r7, #4]
 800317c:	761a      	strb	r2, [r3, #24]
  /*!< Byte 10 */
  SD_csd->DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;
 800317e:	687b      	ldr	r3, [r7, #4]
 8003180:	7e1b      	ldrb	r3, [r3, #24]
 8003182:	b2da      	uxtb	r2, r3
 8003184:	7cbb      	ldrb	r3, [r7, #18]
 8003186:	ea4f 13d3 	mov.w	r3, r3, lsr #7
 800318a:	b2db      	uxtb	r3, r3
 800318c:	ea42 0303 	orr.w	r3, r2, r3
 8003190:	b2da      	uxtb	r2, r3
 8003192:	687b      	ldr	r3, [r7, #4]
 8003194:	761a      	strb	r2, [r3, #24]
    
  SD_csd->EraseGrSize = (CSD_Tab[10] & 0x40) >> 6;
 8003196:	7cbb      	ldrb	r3, [r7, #18]
 8003198:	f003 0340 	and.w	r3, r3, #64	; 0x40
 800319c:	ea4f 13a3 	mov.w	r3, r3, asr #6
 80031a0:	b2da      	uxtb	r2, r3
 80031a2:	687b      	ldr	r3, [r7, #4]
 80031a4:	765a      	strb	r2, [r3, #25]
  SD_csd->EraseGrMul = (CSD_Tab[10] & 0x3F) << 1;
 80031a6:	7cbb      	ldrb	r3, [r7, #18]
 80031a8:	f003 033f 	and.w	r3, r3, #63	; 0x3f
 80031ac:	b2db      	uxtb	r3, r3
 80031ae:	ea4f 0343 	mov.w	r3, r3, lsl #1
 80031b2:	b2da      	uxtb	r2, r3
 80031b4:	687b      	ldr	r3, [r7, #4]
 80031b6:	769a      	strb	r2, [r3, #26]

  /*!< Byte 11 */
  SD_csd->EraseGrMul |= (CSD_Tab[11] & 0x80) >> 7;
 80031b8:	687b      	ldr	r3, [r7, #4]
 80031ba:	7e9b      	ldrb	r3, [r3, #26]
 80031bc:	b2da      	uxtb	r2, r3
 80031be:	7cfb      	ldrb	r3, [r7, #19]
 80031c0:	ea4f 13d3 	mov.w	r3, r3, lsr #7
 80031c4:	b2db      	uxtb	r3, r3
 80031c6:	ea42 0303 	orr.w	r3, r2, r3
 80031ca:	b2da      	uxtb	r2, r3
 80031cc:	687b      	ldr	r3, [r7, #4]
 80031ce:	769a      	strb	r2, [r3, #26]
  SD_csd->WrProtectGrSize = (CSD_Tab[11] & 0x7F);
 80031d0:	7cfb      	ldrb	r3, [r7, #19]
 80031d2:	f003 037f 	and.w	r3, r3, #127	; 0x7f
 80031d6:	b2da      	uxtb	r2, r3
 80031d8:	687b      	ldr	r3, [r7, #4]
 80031da:	76da      	strb	r2, [r3, #27]

  /*!< Byte 12 */
  SD_csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
 80031dc:	7d3b      	ldrb	r3, [r7, #20]
 80031de:	ea4f 13d3 	mov.w	r3, r3, lsr #7
 80031e2:	b2da      	uxtb	r2, r3
 80031e4:	687b      	ldr	r3, [r7, #4]
 80031e6:	771a      	strb	r2, [r3, #28]
  SD_csd->ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
 80031e8:	7d3b      	ldrb	r3, [r7, #20]
 80031ea:	f003 0360 	and.w	r3, r3, #96	; 0x60
 80031ee:	ea4f 1363 	mov.w	r3, r3, asr #5
 80031f2:	b2da      	uxtb	r2, r3
 80031f4:	687b      	ldr	r3, [r7, #4]
 80031f6:	775a      	strb	r2, [r3, #29]
  SD_csd->WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
 80031f8:	7d3b      	ldrb	r3, [r7, #20]
 80031fa:	f003 031c 	and.w	r3, r3, #28
 80031fe:	ea4f 03a3 	mov.w	r3, r3, asr #2
 8003202:	b2da      	uxtb	r2, r3
 8003204:	687b      	ldr	r3, [r7, #4]
 8003206:	779a      	strb	r2, [r3, #30]
  SD_csd->MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;
 8003208:	7d3b      	ldrb	r3, [r7, #20]
 800320a:	f003 0303 	and.w	r3, r3, #3
 800320e:	b2db      	uxtb	r3, r3
 8003210:	ea4f 0383 	mov.w	r3, r3, lsl #2
 8003214:	b2da      	uxtb	r2, r3
 8003216:	687b      	ldr	r3, [r7, #4]
 8003218:	77da      	strb	r2, [r3, #31]

  /*!< Byte 13 */
  SD_csd->MaxWrBlockLen |= (CSD_Tab[13] & 0xC0) >> 6;
 800321a:	687b      	ldr	r3, [r7, #4]
 800321c:	7fdb      	ldrb	r3, [r3, #31]
 800321e:	b2da      	uxtb	r2, r3
 8003220:	7d7b      	ldrb	r3, [r7, #21]
 8003222:	ea4f 1393 	mov.w	r3, r3, lsr #6
 8003226:	b2db      	uxtb	r3, r3
 8003228:	ea42 0303 	orr.w	r3, r2, r3
 800322c:	b2da      	uxtb	r2, r3
 800322e:	687b      	ldr	r3, [r7, #4]
 8003230:	77da      	strb	r2, [r3, #31]
  SD_csd->WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
 8003232:	7d7b      	ldrb	r3, [r7, #21]
 8003234:	f003 0320 	and.w	r3, r3, #32
 8003238:	ea4f 1363 	mov.w	r3, r3, asr #5
 800323c:	b2da      	uxtb	r2, r3
 800323e:	687b      	ldr	r3, [r7, #4]
 8003240:	f883 2020 	strb.w	r2, [r3, #32]
  SD_csd->Reserved3 = 0;
 8003244:	687b      	ldr	r3, [r7, #4]
 8003246:	f04f 0200 	mov.w	r2, #0
 800324a:	f883 2021 	strb.w	r2, [r3, #33]	; 0x21
  SD_csd->ContentProtectAppli = (CSD_Tab[13] & 0x01);
 800324e:	7d7b      	ldrb	r3, [r7, #21]
 8003250:	f003 0301 	and.w	r3, r3, #1
 8003254:	b2da      	uxtb	r2, r3
 8003256:	687b      	ldr	r3, [r7, #4]
 8003258:	f883 2022 	strb.w	r2, [r3, #34]	; 0x22

  /*!< Byte 14 */
  SD_csd->FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
 800325c:	7dbb      	ldrb	r3, [r7, #22]
 800325e:	ea4f 13d3 	mov.w	r3, r3, lsr #7
 8003262:	b2da      	uxtb	r2, r3
 8003264:	687b      	ldr	r3, [r7, #4]
 8003266:	f883 2023 	strb.w	r2, [r3, #35]	; 0x23
  SD_csd->CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
 800326a:	7dbb      	ldrb	r3, [r7, #22]
 800326c:	f003 0340 	and.w	r3, r3, #64	; 0x40
 8003270:	ea4f 13a3 	mov.w	r3, r3, asr #6
 8003274:	b2da      	uxtb	r2, r3
 8003276:	687b      	ldr	r3, [r7, #4]
 8003278:	e002      	b.n	8003280 <SD_GetCSDRegister+0x2cc>
 800327a:	bf00      	nop
 800327c:	40010c00 	.word	0x40010c00
 8003280:	f883 2024 	strb.w	r2, [r3, #36]	; 0x24
  SD_csd->PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
 8003284:	7dbb      	ldrb	r3, [r7, #22]
 8003286:	f003 0320 	and.w	r3, r3, #32
 800328a:	ea4f 1363 	mov.w	r3, r3, asr #5
 800328e:	b2da      	uxtb	r2, r3
 8003290:	687b      	ldr	r3, [r7, #4]
 8003292:	f883 2025 	strb.w	r2, [r3, #37]	; 0x25
  SD_csd->TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
 8003296:	7dbb      	ldrb	r3, [r7, #22]
 8003298:	f003 0310 	and.w	r3, r3, #16
 800329c:	ea4f 1323 	mov.w	r3, r3, asr #4
 80032a0:	b2da      	uxtb	r2, r3
 80032a2:	687b      	ldr	r3, [r7, #4]
 80032a4:	f883 2026 	strb.w	r2, [r3, #38]	; 0x26
  SD_csd->FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
 80032a8:	7dbb      	ldrb	r3, [r7, #22]
 80032aa:	f003 030c 	and.w	r3, r3, #12
 80032ae:	ea4f 03a3 	mov.w	r3, r3, asr #2
 80032b2:	b2da      	uxtb	r2, r3
 80032b4:	687b      	ldr	r3, [r7, #4]
 80032b6:	f883 2027 	strb.w	r2, [r3, #39]	; 0x27
  SD_csd->ECC = (CSD_Tab[14] & 0x03);
 80032ba:	7dbb      	ldrb	r3, [r7, #22]
 80032bc:	f003 0303 	and.w	r3, r3, #3
 80032c0:	b2da      	uxtb	r2, r3
 80032c2:	687b      	ldr	r3, [r7, #4]
 80032c4:	f883 2028 	strb.w	r2, [r3, #40]	; 0x28

  /*!< Byte 15 */
  SD_csd->CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
 80032c8:	7dfb      	ldrb	r3, [r7, #23]
 80032ca:	ea4f 0353 	mov.w	r3, r3, lsr #1
 80032ce:	b2da      	uxtb	r2, r3
 80032d0:	687b      	ldr	r3, [r7, #4]
 80032d2:	f883 2029 	strb.w	r2, [r3, #41]	; 0x29
  SD_csd->Reserved4 = 1;
 80032d6:	687b      	ldr	r3, [r7, #4]
 80032d8:	f04f 0201 	mov.w	r2, #1
 80032dc:	f883 202a 	strb.w	r2, [r3, #42]	; 0x2a

  /*!< Return the reponse */
  return rvalue;
 80032e0:	7efb      	ldrb	r3, [r7, #27]
}
 80032e2:	4618      	mov	r0, r3
 80032e4:	f107 0720 	add.w	r7, r7, #32
 80032e8:	46bd      	mov	sp, r7
 80032ea:	bd80      	pop	{r7, pc}

080032ec <SD_GetCIDRegister>:
  * @retval The SD Response: 
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GetCIDRegister(SD_CID* SD_cid)
{
 80032ec:	b580      	push	{r7, lr}
 80032ee:	b088      	sub	sp, #32
 80032f0:	af00      	add	r7, sp, #0
 80032f2:	6078      	str	r0, [r7, #4]
  uint32_t i = 0;
 80032f4:	f04f 0300 	mov.w	r3, #0
 80032f8:	61fb      	str	r3, [r7, #28]
  SD_Error rvalue = SD_RESPONSE_FAILURE;
 80032fa:	f04f 03ff 	mov.w	r3, #255	; 0xff
 80032fe:	76fb      	strb	r3, [r7, #27]
  uint8_t CID_Tab[16];
  
  /*!< SD chip select low */
  SD_CS_LOW();
 8003300:	4860      	ldr	r0, [pc, #384]	; (8003484 <SD_GetCIDRegister+0x198>)
 8003302:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8003306:	f001 f960 	bl	80045ca <GPIO_ResetBits>
  
  /*!< Send CMD10 (CID register) */
  SD_SendCmd(SD_CMD_SEND_CID, 0, 0xFF);
 800330a:	f04f 000a 	mov.w	r0, #10
 800330e:	f04f 0100 	mov.w	r1, #0
 8003312:	f04f 02ff 	mov.w	r2, #255	; 0xff
 8003316:	f000 f8b7 	bl	8003488 <SD_SendCmd>
  
  /*!< Wait for response in the R1 format (0x00 is no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
 800331a:	f04f 0000 	mov.w	r0, #0
 800331e:	f000 f92b 	bl	8003578 <SD_GetResponse>
 8003322:	4603      	mov	r3, r0
 8003324:	2b00      	cmp	r3, #0
 8003326:	d125      	bne.n	8003374 <SD_GetCIDRegister+0x88>
  {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
 8003328:	f04f 00fe 	mov.w	r0, #254	; 0xfe
 800332c:	f000 f924 	bl	8003578 <SD_GetResponse>
 8003330:	4603      	mov	r3, r0
 8003332:	2b00      	cmp	r3, #0
 8003334:	d113      	bne.n	800335e <SD_GetCIDRegister+0x72>
    {
      /*!< Store CID register value on CID_Tab */
      for (i = 0; i < 16; i++)
 8003336:	f04f 0300 	mov.w	r3, #0
 800333a:	61fb      	str	r3, [r7, #28]
 800333c:	e00c      	b.n	8003358 <SD_GetCIDRegister+0x6c>
      {
        CID_Tab[i] = SD_ReadByte();
 800333e:	f000 f9e1 	bl	8003704 <SD_ReadByte>
 8003342:	4603      	mov	r3, r0
 8003344:	461a      	mov	r2, r3
 8003346:	f107 0108 	add.w	r1, r7, #8
 800334a:	69fb      	ldr	r3, [r7, #28]
 800334c:	18cb      	adds	r3, r1, r3
 800334e:	701a      	strb	r2, [r3, #0]
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
  {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      /*!< Store CID register value on CID_Tab */
      for (i = 0; i < 16; i++)
 8003350:	69fb      	ldr	r3, [r7, #28]
 8003352:	f103 0301 	add.w	r3, r3, #1
 8003356:	61fb      	str	r3, [r7, #28]
 8003358:	69fb      	ldr	r3, [r7, #28]
 800335a:	2b0f      	cmp	r3, #15
 800335c:	d9ef      	bls.n	800333e <SD_GetCIDRegister+0x52>
      {
        CID_Tab[i] = SD_ReadByte();
      }
    }
    /*!< Get CRC bytes (not really needed by us, but required by SD) */
    SD_WriteByte(SD_DUMMY_BYTE);
 800335e:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8003362:	f000 f9a5 	bl	80036b0 <SD_WriteByte>
    SD_WriteByte(SD_DUMMY_BYTE);
 8003366:	f04f 00ff 	mov.w	r0, #255	; 0xff
 800336a:	f000 f9a1 	bl	80036b0 <SD_WriteByte>
    /*!< Set response value to success */
    rvalue = SD_RESPONSE_NO_ERROR;
 800336e:	f04f 0300 	mov.w	r3, #0
 8003372:	76fb      	strb	r3, [r7, #27]
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
 8003374:	4843      	ldr	r0, [pc, #268]	; (8003484 <SD_GetCIDRegister+0x198>)
 8003376:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 800337a:	f001 f924 	bl	80045c6 <GPIO_SetBits>
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
 800337e:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8003382:	f000 f995 	bl	80036b0 <SD_WriteByte>

  /*!< Byte 0 */
  SD_cid->ManufacturerID = CID_Tab[0];
 8003386:	7a3a      	ldrb	r2, [r7, #8]
 8003388:	687b      	ldr	r3, [r7, #4]
 800338a:	701a      	strb	r2, [r3, #0]

  /*!< Byte 1 */
  SD_cid->OEM_AppliID = CID_Tab[1] << 8;
 800338c:	7a7b      	ldrb	r3, [r7, #9]
 800338e:	ea4f 2303 	mov.w	r3, r3, lsl #8
 8003392:	b29a      	uxth	r2, r3
 8003394:	687b      	ldr	r3, [r7, #4]
 8003396:	805a      	strh	r2, [r3, #2]

  /*!< Byte 2 */
  SD_cid->OEM_AppliID |= CID_Tab[2];
 8003398:	687b      	ldr	r3, [r7, #4]
 800339a:	885b      	ldrh	r3, [r3, #2]
 800339c:	b29a      	uxth	r2, r3
 800339e:	7abb      	ldrb	r3, [r7, #10]
 80033a0:	ea42 0303 	orr.w	r3, r2, r3
 80033a4:	b29a      	uxth	r2, r3
 80033a6:	687b      	ldr	r3, [r7, #4]
 80033a8:	805a      	strh	r2, [r3, #2]

  /*!< Byte 3 */
  SD_cid->ProdName1 = CID_Tab[3] << 24;
 80033aa:	7afb      	ldrb	r3, [r7, #11]
 80033ac:	ea4f 6303 	mov.w	r3, r3, lsl #24
 80033b0:	461a      	mov	r2, r3
 80033b2:	687b      	ldr	r3, [r7, #4]
 80033b4:	605a      	str	r2, [r3, #4]

  /*!< Byte 4 */
  SD_cid->ProdName1 |= CID_Tab[4] << 16;
 80033b6:	687b      	ldr	r3, [r7, #4]
 80033b8:	685a      	ldr	r2, [r3, #4]
 80033ba:	7b3b      	ldrb	r3, [r7, #12]
 80033bc:	ea4f 4303 	mov.w	r3, r3, lsl #16
 80033c0:	431a      	orrs	r2, r3
 80033c2:	687b      	ldr	r3, [r7, #4]
 80033c4:	605a      	str	r2, [r3, #4]

  /*!< Byte 5 */
  SD_cid->ProdName1 |= CID_Tab[5] << 8;
 80033c6:	687b      	ldr	r3, [r7, #4]
 80033c8:	685a      	ldr	r2, [r3, #4]
 80033ca:	7b7b      	ldrb	r3, [r7, #13]
 80033cc:	ea4f 2303 	mov.w	r3, r3, lsl #8
 80033d0:	431a      	orrs	r2, r3
 80033d2:	687b      	ldr	r3, [r7, #4]
 80033d4:	605a      	str	r2, [r3, #4]

  /*!< Byte 6 */
  SD_cid->ProdName1 |= CID_Tab[6];
 80033d6:	687b      	ldr	r3, [r7, #4]
 80033d8:	685a      	ldr	r2, [r3, #4]
 80033da:	7bbb      	ldrb	r3, [r7, #14]
 80033dc:	431a      	orrs	r2, r3
 80033de:	687b      	ldr	r3, [r7, #4]
 80033e0:	605a      	str	r2, [r3, #4]

  /*!< Byte 7 */
  SD_cid->ProdName2 = CID_Tab[7];
 80033e2:	7bfa      	ldrb	r2, [r7, #15]
 80033e4:	687b      	ldr	r3, [r7, #4]
 80033e6:	721a      	strb	r2, [r3, #8]

  /*!< Byte 8 */
  SD_cid->ProdRev = CID_Tab[8];
 80033e8:	7c3a      	ldrb	r2, [r7, #16]
 80033ea:	687b      	ldr	r3, [r7, #4]
 80033ec:	725a      	strb	r2, [r3, #9]

  /*!< Byte 9 */
  SD_cid->ProdSN = CID_Tab[9] << 24;
 80033ee:	7c7b      	ldrb	r3, [r7, #17]
 80033f0:	ea4f 6303 	mov.w	r3, r3, lsl #24
 80033f4:	461a      	mov	r2, r3
 80033f6:	687b      	ldr	r3, [r7, #4]
 80033f8:	60da      	str	r2, [r3, #12]

  /*!< Byte 10 */
  SD_cid->ProdSN |= CID_Tab[10] << 16;
 80033fa:	687b      	ldr	r3, [r7, #4]
 80033fc:	68da      	ldr	r2, [r3, #12]
 80033fe:	7cbb      	ldrb	r3, [r7, #18]
 8003400:	ea4f 4303 	mov.w	r3, r3, lsl #16
 8003404:	431a      	orrs	r2, r3
 8003406:	687b      	ldr	r3, [r7, #4]
 8003408:	60da      	str	r2, [r3, #12]

  /*!< Byte 11 */
  SD_cid->ProdSN |= CID_Tab[11] << 8;
 800340a:	687b      	ldr	r3, [r7, #4]
 800340c:	68da      	ldr	r2, [r3, #12]
 800340e:	7cfb      	ldrb	r3, [r7, #19]
 8003410:	ea4f 2303 	mov.w	r3, r3, lsl #8
 8003414:	431a      	orrs	r2, r3
 8003416:	687b      	ldr	r3, [r7, #4]
 8003418:	60da      	str	r2, [r3, #12]

  /*!< Byte 12 */
  SD_cid->ProdSN |= CID_Tab[12];
 800341a:	687b      	ldr	r3, [r7, #4]
 800341c:	68da      	ldr	r2, [r3, #12]
 800341e:	7d3b      	ldrb	r3, [r7, #20]
 8003420:	431a      	orrs	r2, r3
 8003422:	687b      	ldr	r3, [r7, #4]
 8003424:	60da      	str	r2, [r3, #12]

  /*!< Byte 13 */
  SD_cid->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
 8003426:	687b      	ldr	r3, [r7, #4]
 8003428:	7c1b      	ldrb	r3, [r3, #16]
 800342a:	b2da      	uxtb	r2, r3
 800342c:	7d7b      	ldrb	r3, [r7, #21]
 800342e:	ea4f 1313 	mov.w	r3, r3, lsr #4
 8003432:	b2db      	uxtb	r3, r3
 8003434:	ea42 0303 	orr.w	r3, r2, r3
 8003438:	b2da      	uxtb	r2, r3
 800343a:	687b      	ldr	r3, [r7, #4]
 800343c:	741a      	strb	r2, [r3, #16]
  SD_cid->ManufactDate = (CID_Tab[13] & 0x0F) << 8;
 800343e:	7d7b      	ldrb	r3, [r7, #21]
 8003440:	f003 030f 	and.w	r3, r3, #15
 8003444:	b29b      	uxth	r3, r3
 8003446:	ea4f 2303 	mov.w	r3, r3, lsl #8
 800344a:	b29a      	uxth	r2, r3
 800344c:	687b      	ldr	r3, [r7, #4]
 800344e:	825a      	strh	r2, [r3, #18]

  /*!< Byte 14 */
  SD_cid->ManufactDate |= CID_Tab[14];
 8003450:	687b      	ldr	r3, [r7, #4]
 8003452:	8a5b      	ldrh	r3, [r3, #18]
 8003454:	b29a      	uxth	r2, r3
 8003456:	7dbb      	ldrb	r3, [r7, #22]
 8003458:	ea42 0303 	orr.w	r3, r2, r3
 800345c:	b29a      	uxth	r2, r3
 800345e:	687b      	ldr	r3, [r7, #4]
 8003460:	825a      	strh	r2, [r3, #18]

  /*!< Byte 15 */
  SD_cid->CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
 8003462:	7dfb      	ldrb	r3, [r7, #23]
 8003464:	ea4f 0353 	mov.w	r3, r3, lsr #1
 8003468:	b2da      	uxtb	r2, r3
 800346a:	687b      	ldr	r3, [r7, #4]
 800346c:	751a      	strb	r2, [r3, #20]
  SD_cid->Reserved2 = 1;
 800346e:	687b      	ldr	r3, [r7, #4]
 8003470:	f04f 0201 	mov.w	r2, #1
 8003474:	755a      	strb	r2, [r3, #21]

  /*!< Return the reponse */
  return rvalue;
 8003476:	7efb      	ldrb	r3, [r7, #27]
}
 8003478:	4618      	mov	r0, r3
 800347a:	f107 0720 	add.w	r7, r7, #32
 800347e:	46bd      	mov	sp, r7
 8003480:	bd80      	pop	{r7, pc}
 8003482:	bf00      	nop
 8003484:	40010c00 	.word	0x40010c00

08003488 <SD_SendCmd>:
  * @param  Arg: The command argument.
  * @param  Crc: The CRC.
  * @retval None
  */
void SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc)
{
 8003488:	b580      	push	{r7, lr}
 800348a:	b088      	sub	sp, #32
 800348c:	af00      	add	r7, sp, #0
 800348e:	60b9      	str	r1, [r7, #8]
 8003490:	4613      	mov	r3, r2
 8003492:	4602      	mov	r2, r0
 8003494:	73fa      	strb	r2, [r7, #15]
 8003496:	71fb      	strb	r3, [r7, #7]
  uint32_t i = 0x00;
 8003498:	f04f 0300 	mov.w	r3, #0
 800349c:	61fb      	str	r3, [r7, #28]
  
  uint8_t Frame[6];
  
  Frame[0] = (Cmd | 0x40); /*!< Construct byte 1 */
 800349e:	7bfb      	ldrb	r3, [r7, #15]
 80034a0:	f043 0340 	orr.w	r3, r3, #64	; 0x40
 80034a4:	b2db      	uxtb	r3, r3
 80034a6:	753b      	strb	r3, [r7, #20]
  
  Frame[1] = (uint8_t)(Arg >> 24); /*!< Construct byte 2 */
 80034a8:	68bb      	ldr	r3, [r7, #8]
 80034aa:	ea4f 6313 	mov.w	r3, r3, lsr #24
 80034ae:	b2db      	uxtb	r3, r3
 80034b0:	757b      	strb	r3, [r7, #21]
  
  Frame[2] = (uint8_t)(Arg >> 16); /*!< Construct byte 3 */
 80034b2:	68bb      	ldr	r3, [r7, #8]
 80034b4:	ea4f 4313 	mov.w	r3, r3, lsr #16
 80034b8:	b2db      	uxtb	r3, r3
 80034ba:	75bb      	strb	r3, [r7, #22]
  
  Frame[3] = (uint8_t)(Arg >> 8); /*!< Construct byte 4 */
 80034bc:	68bb      	ldr	r3, [r7, #8]
 80034be:	ea4f 2313 	mov.w	r3, r3, lsr #8
 80034c2:	b2db      	uxtb	r3, r3
 80034c4:	75fb      	strb	r3, [r7, #23]
  
  Frame[4] = (uint8_t)(Arg); /*!< Construct byte 5 */
 80034c6:	68bb      	ldr	r3, [r7, #8]
 80034c8:	b2db      	uxtb	r3, r3
 80034ca:	763b      	strb	r3, [r7, #24]
  
  Frame[5] = (Crc); /*!< Construct CRC: byte 6 */
 80034cc:	79fb      	ldrb	r3, [r7, #7]
 80034ce:	767b      	strb	r3, [r7, #25]
  
  for (i = 0; i < 6; i++)
 80034d0:	f04f 0300 	mov.w	r3, #0
 80034d4:	61fb      	str	r3, [r7, #28]
 80034d6:	e00b      	b.n	80034f0 <SD_SendCmd+0x68>
  {
    SD_WriteByte(Frame[i]); /*!< Send the Cmd bytes */
 80034d8:	f107 0214 	add.w	r2, r7, #20
 80034dc:	69fb      	ldr	r3, [r7, #28]
 80034de:	18d3      	adds	r3, r2, r3
 80034e0:	781b      	ldrb	r3, [r3, #0]
 80034e2:	4618      	mov	r0, r3
 80034e4:	f000 f8e4 	bl	80036b0 <SD_WriteByte>
  
  Frame[4] = (uint8_t)(Arg); /*!< Construct byte 5 */
  
  Frame[5] = (Crc); /*!< Construct CRC: byte 6 */
  
  for (i = 0; i < 6; i++)
 80034e8:	69fb      	ldr	r3, [r7, #28]
 80034ea:	f103 0301 	add.w	r3, r3, #1
 80034ee:	61fb      	str	r3, [r7, #28]
 80034f0:	69fb      	ldr	r3, [r7, #28]
 80034f2:	2b05      	cmp	r3, #5
 80034f4:	d9f0      	bls.n	80034d8 <SD_SendCmd+0x50>
  {
    SD_WriteByte(Frame[i]); /*!< Send the Cmd bytes */
  }
}
 80034f6:	f107 0720 	add.w	r7, r7, #32
 80034fa:	46bd      	mov	sp, r7
 80034fc:	bd80      	pop	{r7, pc}
 80034fe:	bf00      	nop

08003500 <SD_GetDataResponse>:
  *         - status 101: Data rejected due to a crc error
  *         - status 110: Data rejected due to a Write error.
  *         - status 111: Data rejected due to other error.
  */
uint8_t SD_GetDataResponse(void)
{
 8003500:	b580      	push	{r7, lr}
 8003502:	b082      	sub	sp, #8
 8003504:	af00      	add	r7, sp, #0
  uint32_t i = 0;
 8003506:	f04f 0300 	mov.w	r3, #0
 800350a:	607b      	str	r3, [r7, #4]
  uint8_t response, rvalue;

  while (i <= 64)
 800350c:	e025      	b.n	800355a <SD_GetDataResponse+0x5a>
  {
    /*!< Read resonse */
    response = SD_ReadByte();
 800350e:	f000 f8f9 	bl	8003704 <SD_ReadByte>
 8003512:	4603      	mov	r3, r0
 8003514:	70fb      	strb	r3, [r7, #3]
    /*!< Mask unused bits */
    response &= 0x1F;
 8003516:	78fb      	ldrb	r3, [r7, #3]
 8003518:	f003 031f 	and.w	r3, r3, #31
 800351c:	70fb      	strb	r3, [r7, #3]
    switch (response)
 800351e:	78fb      	ldrb	r3, [r7, #3]
 8003520:	2b0b      	cmp	r3, #11
 8003522:	d007      	beq.n	8003534 <SD_GetDataResponse+0x34>
 8003524:	2b0d      	cmp	r3, #13
 8003526:	d008      	beq.n	800353a <SD_GetDataResponse+0x3a>
 8003528:	2b05      	cmp	r3, #5
 800352a:	d109      	bne.n	8003540 <SD_GetDataResponse+0x40>
    {
      case SD_DATA_OK:
      {
        rvalue = SD_DATA_OK;
 800352c:	f04f 0305 	mov.w	r3, #5
 8003530:	70bb      	strb	r3, [r7, #2]
        break;
 8003532:	e009      	b.n	8003548 <SD_GetDataResponse+0x48>
      }
      case SD_DATA_CRC_ERROR:
        return SD_DATA_CRC_ERROR;
 8003534:	f04f 030b 	mov.w	r3, #11
 8003538:	e019      	b.n	800356e <SD_GetDataResponse+0x6e>
      case SD_DATA_WRITE_ERROR:
        return SD_DATA_WRITE_ERROR;
 800353a:	f04f 030d 	mov.w	r3, #13
 800353e:	e016      	b.n	800356e <SD_GetDataResponse+0x6e>
      default:
      {
        rvalue = SD_DATA_OTHER_ERROR;
 8003540:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8003544:	70bb      	strb	r3, [r7, #2]
        break;
 8003546:	bf00      	nop
      }
    }
    /*!< Exit loop in case of data ok */
    if (rvalue == SD_DATA_OK)
 8003548:	78bb      	ldrb	r3, [r7, #2]
 800354a:	2b05      	cmp	r3, #5
 800354c:	d101      	bne.n	8003552 <SD_GetDataResponse+0x52>
      break;
 800354e:	bf00      	nop
    /*!< Increment loop counter */
    i++;
  }

  /*!< Wait null data */
  while (SD_ReadByte() == 0);
 8003550:	e007      	b.n	8003562 <SD_GetDataResponse+0x62>
    }
    /*!< Exit loop in case of data ok */
    if (rvalue == SD_DATA_OK)
      break;
    /*!< Increment loop counter */
    i++;
 8003552:	687b      	ldr	r3, [r7, #4]
 8003554:	f103 0301 	add.w	r3, r3, #1
 8003558:	607b      	str	r3, [r7, #4]
uint8_t SD_GetDataResponse(void)
{
  uint32_t i = 0;
  uint8_t response, rvalue;

  while (i <= 64)
 800355a:	687b      	ldr	r3, [r7, #4]
 800355c:	2b40      	cmp	r3, #64	; 0x40
 800355e:	d9d6      	bls.n	800350e <SD_GetDataResponse+0xe>
    /*!< Increment loop counter */
    i++;
  }

  /*!< Wait null data */
  while (SD_ReadByte() == 0);
 8003560:	bf00      	nop
 8003562:	f000 f8cf 	bl	8003704 <SD_ReadByte>
 8003566:	4603      	mov	r3, r0
 8003568:	2b00      	cmp	r3, #0
 800356a:	d0fa      	beq.n	8003562 <SD_GetDataResponse+0x62>

  /*!< Return response */
  return response;
 800356c:	78fb      	ldrb	r3, [r7, #3]
}
 800356e:	4618      	mov	r0, r3
 8003570:	f107 0708 	add.w	r7, r7, #8
 8003574:	46bd      	mov	sp, r7
 8003576:	bd80      	pop	{r7, pc}

08003578 <SD_GetResponse>:
  * @retval The SD Response: 
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GetResponse(uint8_t Response)
{
 8003578:	b580      	push	{r7, lr}
 800357a:	b084      	sub	sp, #16
 800357c:	af00      	add	r7, sp, #0
 800357e:	4603      	mov	r3, r0
 8003580:	71fb      	strb	r3, [r7, #7]
  uint32_t Count = 0xFFF;
 8003582:	f640 73ff 	movw	r3, #4095	; 0xfff
 8003586:	60fb      	str	r3, [r7, #12]

  /*!< Check if response is got or a timeout is happen */
  while ((SD_ReadByte() != Response) && Count)
 8003588:	e003      	b.n	8003592 <SD_GetResponse+0x1a>
  {
    Count--;
 800358a:	68fb      	ldr	r3, [r7, #12]
 800358c:	f103 33ff 	add.w	r3, r3, #4294967295
 8003590:	60fb      	str	r3, [r7, #12]
SD_Error SD_GetResponse(uint8_t Response)
{
  uint32_t Count = 0xFFF;

  /*!< Check if response is got or a timeout is happen */
  while ((SD_ReadByte() != Response) && Count)
 8003592:	f000 f8b7 	bl	8003704 <SD_ReadByte>
 8003596:	4603      	mov	r3, r0
 8003598:	79fa      	ldrb	r2, [r7, #7]
 800359a:	429a      	cmp	r2, r3
 800359c:	d002      	beq.n	80035a4 <SD_GetResponse+0x2c>
 800359e:	68fb      	ldr	r3, [r7, #12]
 80035a0:	2b00      	cmp	r3, #0
 80035a2:	d1f2      	bne.n	800358a <SD_GetResponse+0x12>
  {
    Count--;
  }
  if (Count == 0)
 80035a4:	68fb      	ldr	r3, [r7, #12]
 80035a6:	2b00      	cmp	r3, #0
 80035a8:	d102      	bne.n	80035b0 <SD_GetResponse+0x38>
  {
    /*!< After time out */
    return SD_RESPONSE_FAILURE;
 80035aa:	f04f 03ff 	mov.w	r3, #255	; 0xff
 80035ae:	e001      	b.n	80035b4 <SD_GetResponse+0x3c>
  }
  else
  {
    /*!< Right response got */
    return SD_RESPONSE_NO_ERROR;
 80035b0:	f04f 0300 	mov.w	r3, #0
  }
}
 80035b4:	4618      	mov	r0, r3
 80035b6:	f107 0710 	add.w	r7, r7, #16
 80035ba:	46bd      	mov	sp, r7
 80035bc:	bd80      	pop	{r7, pc}
 80035be:	bf00      	nop

080035c0 <SD_GetStatus>:
  * @brief  Returns the SD status.
  * @param  None
  * @retval The SD status.
  */
uint16_t SD_GetStatus(void)
{
 80035c0:	b580      	push	{r7, lr}
 80035c2:	b082      	sub	sp, #8
 80035c4:	af00      	add	r7, sp, #0
  uint16_t Status = 0;
 80035c6:	f04f 0300 	mov.w	r3, #0
 80035ca:	80fb      	strh	r3, [r7, #6]

  /*!< SD chip select low */
  SD_CS_LOW();
 80035cc:	4814      	ldr	r0, [pc, #80]	; (8003620 <SD_GetStatus+0x60>)
 80035ce:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 80035d2:	f000 fffa 	bl	80045ca <GPIO_ResetBits>

  /*!< Send CMD13 (SD_SEND_STATUS) to get SD status */
  SD_SendCmd(SD_CMD_SEND_STATUS, 0, 0xFF);
 80035d6:	f04f 000d 	mov.w	r0, #13
 80035da:	f04f 0100 	mov.w	r1, #0
 80035de:	f04f 02ff 	mov.w	r2, #255	; 0xff
 80035e2:	f7ff ff51 	bl	8003488 <SD_SendCmd>

  Status = SD_ReadByte();
 80035e6:	f000 f88d 	bl	8003704 <SD_ReadByte>
 80035ea:	4603      	mov	r3, r0
 80035ec:	80fb      	strh	r3, [r7, #6]
  Status |= (uint16_t)(SD_ReadByte() << 8);
 80035ee:	f000 f889 	bl	8003704 <SD_ReadByte>
 80035f2:	4603      	mov	r3, r0
 80035f4:	ea4f 2303 	mov.w	r3, r3, lsl #8
 80035f8:	b29a      	uxth	r2, r3
 80035fa:	88fb      	ldrh	r3, [r7, #6]
 80035fc:	ea42 0303 	orr.w	r3, r2, r3
 8003600:	80fb      	strh	r3, [r7, #6]

  /*!< SD chip select high */
  SD_CS_HIGH();
 8003602:	4807      	ldr	r0, [pc, #28]	; (8003620 <SD_GetStatus+0x60>)
 8003604:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8003608:	f000 ffdd 	bl	80045c6 <GPIO_SetBits>

  /*!< Send dummy byte 0xFF */
  SD_WriteByte(SD_DUMMY_BYTE);
 800360c:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8003610:	f000 f84e 	bl	80036b0 <SD_WriteByte>

  return Status;
 8003614:	88fb      	ldrh	r3, [r7, #6]
}
 8003616:	4618      	mov	r0, r3
 8003618:	f107 0708 	add.w	r7, r7, #8
 800361c:	46bd      	mov	sp, r7
 800361e:	bd80      	pop	{r7, pc}
 8003620:	40010c00 	.word	0x40010c00

08003624 <SD_GoIdleState>:
  * @retval The SD Response: 
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GoIdleState(void)
{
 8003624:	b580      	push	{r7, lr}
 8003626:	af00      	add	r7, sp, #0
  /*!< SD chip select low */
  SD_CS_LOW();
 8003628:	4820      	ldr	r0, [pc, #128]	; (80036ac <SD_GoIdleState+0x88>)
 800362a:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 800362e:	f000 ffcc 	bl	80045ca <GPIO_ResetBits>
  
  /*!< Send CMD0 (SD_CMD_GO_IDLE_STATE) to put SD in SPI mode */
  SD_SendCmd(SD_CMD_GO_IDLE_STATE, 0, 0x95);
 8003632:	f04f 0000 	mov.w	r0, #0
 8003636:	f04f 0100 	mov.w	r1, #0
 800363a:	f04f 0295 	mov.w	r2, #149	; 0x95
 800363e:	f7ff ff23 	bl	8003488 <SD_SendCmd>
  
  /*!< Wait for In Idle State Response (R1 Format) equal to 0x01 */
  if (SD_GetResponse(SD_IN_IDLE_STATE))
 8003642:	f04f 0001 	mov.w	r0, #1
 8003646:	f7ff ff97 	bl	8003578 <SD_GetResponse>
 800364a:	4603      	mov	r3, r0
 800364c:	2b00      	cmp	r3, #0
 800364e:	d002      	beq.n	8003656 <SD_GoIdleState+0x32>
  {
    /*!< No Idle State Response: return response failue */
    return SD_RESPONSE_FAILURE;
 8003650:	f04f 03ff 	mov.w	r3, #255	; 0xff
 8003654:	e027      	b.n	80036a6 <SD_GoIdleState+0x82>
  }
  /*----------Activates the card initialization process-----------*/
  do
  {
    /*!< SD chip select high */
    SD_CS_HIGH();
 8003656:	4815      	ldr	r0, [pc, #84]	; (80036ac <SD_GoIdleState+0x88>)
 8003658:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 800365c:	f000 ffb3 	bl	80045c6 <GPIO_SetBits>
    
    /*!< Send Dummy byte 0xFF */
    SD_WriteByte(SD_DUMMY_BYTE);
 8003660:	f04f 00ff 	mov.w	r0, #255	; 0xff
 8003664:	f000 f824 	bl	80036b0 <SD_WriteByte>
    
    /*!< SD chip select low */
    SD_CS_LOW();
 8003668:	4810      	ldr	r0, [pc, #64]	; (80036ac <SD_GoIdleState+0x88>)
 800366a:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 800366e:	f000 ffac 	bl	80045ca <GPIO_ResetBits>
    
    /*!< Send CMD1 (Activates the card process) until response equal to 0x0 */
    SD_SendCmd(SD_CMD_SEND_OP_COND, 0, 0xFF);
 8003672:	f04f 0001 	mov.w	r0, #1
 8003676:	f04f 0100 	mov.w	r1, #0
 800367a:	f04f 02ff 	mov.w	r2, #255	; 0xff
 800367e:	f7ff ff03 	bl	8003488 <SD_SendCmd>
    /*!< Wait for no error Response (R1 Format) equal to 0x00 */
  }
  while (SD_GetResponse(SD_RESPONSE_NO_ERROR));
 8003682:	f04f 0000 	mov.w	r0, #0
 8003686:	f7ff ff77 	bl	8003578 <SD_GetResponse>
 800368a:	4603      	mov	r3, r0
 800368c:	2b00      	cmp	r3, #0
 800368e:	d1e2      	bne.n	8003656 <SD_GoIdleState+0x32>
  
  /*!< SD chip select high */
  SD_CS_HIGH();
 8003690:	4806      	ldr	r0, [pc, #24]	; (80036ac <SD_GoIdleState+0x88>)
 8003692:	f44f 4180 	mov.w	r1, #16384	; 0x4000
 8003696:	f000 ff96 	bl	80045c6 <GPIO_SetBits>
  
  /*!< Send dummy byte 0xFF */
  SD_WriteByte(SD_DUMMY_BYTE);
 800369a:	f04f 00ff 	mov.w	r0, #255	; 0xff
 800369e:	f000 f807 	bl	80036b0 <SD_WriteByte>
  
  return SD_RESPONSE_NO_ERROR;
 80036a2:	f04f 0300 	mov.w	r3, #0
}
 80036a6:	4618      	mov	r0, r3
 80036a8:	bd80      	pop	{r7, pc}
 80036aa:	bf00      	nop
 80036ac:	40010c00 	.word	0x40010c00

080036b0 <SD_WriteByte>:
  * @brief  Write a byte on the SD.
  * @param  Data: byte to send.
  * @retval None
  */
uint8_t SD_WriteByte(uint8_t Data)
{
 80036b0:	b580      	push	{r7, lr}
 80036b2:	b082      	sub	sp, #8
 80036b4:	af00      	add	r7, sp, #0
 80036b6:	4603      	mov	r3, r0
 80036b8:	71fb      	strb	r3, [r7, #7]
  /*!< Wait until the transmit buffer is empty */
  while(SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_TXE) == RESET)
 80036ba:	bf00      	nop
 80036bc:	4810      	ldr	r0, [pc, #64]	; (8003700 <SD_WriteByte+0x50>)
 80036be:	f04f 0102 	mov.w	r1, #2
 80036c2:	f001 fb51 	bl	8004d68 <SPI_I2S_GetFlagStatus>
 80036c6:	4603      	mov	r3, r0
 80036c8:	2b00      	cmp	r3, #0
 80036ca:	d0f7      	beq.n	80036bc <SD_WriteByte+0xc>
  {
  }
  
  /*!< Send the byte */
  SPI_I2S_SendData(SD_SPI, Data);
 80036cc:	79fb      	ldrb	r3, [r7, #7]
 80036ce:	b29b      	uxth	r3, r3
 80036d0:	480b      	ldr	r0, [pc, #44]	; (8003700 <SD_WriteByte+0x50>)
 80036d2:	4619      	mov	r1, r3
 80036d4:	f001 faed 	bl	8004cb2 <SPI_I2S_SendData>
  
  /*!< Wait to receive a byte*/
  while(SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
 80036d8:	bf00      	nop
 80036da:	4809      	ldr	r0, [pc, #36]	; (8003700 <SD_WriteByte+0x50>)
 80036dc:	f04f 0101 	mov.w	r1, #1
 80036e0:	f001 fb42 	bl	8004d68 <SPI_I2S_GetFlagStatus>
 80036e4:	4603      	mov	r3, r0
 80036e6:	2b00      	cmp	r3, #0
 80036e8:	d0f7      	beq.n	80036da <SD_WriteByte+0x2a>
  {
  }
  
  /*!< Return the byte read from the SPI bus */ 
  return SPI_I2S_ReceiveData(SD_SPI);
 80036ea:	4805      	ldr	r0, [pc, #20]	; (8003700 <SD_WriteByte+0x50>)
 80036ec:	f001 fae3 	bl	8004cb6 <SPI_I2S_ReceiveData>
 80036f0:	4603      	mov	r3, r0
 80036f2:	b2db      	uxtb	r3, r3
}
 80036f4:	4618      	mov	r0, r3
 80036f6:	f107 0708 	add.w	r7, r7, #8
 80036fa:	46bd      	mov	sp, r7
 80036fc:	bd80      	pop	{r7, pc}
 80036fe:	bf00      	nop
 8003700:	40013000 	.word	0x40013000

08003704 <SD_ReadByte>:
  * @brief  Read a byte from the SD.
  * @param  None
  * @retval The received byte.
  */
uint8_t SD_ReadByte(void)
{
 8003704:	b580      	push	{r7, lr}
 8003706:	b082      	sub	sp, #8
 8003708:	af00      	add	r7, sp, #0
  uint8_t Data = 0;
 800370a:	f04f 0300 	mov.w	r3, #0
 800370e:	71fb      	strb	r3, [r7, #7]
  
  /*!< Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_TXE) == RESET)
 8003710:	bf00      	nop
 8003712:	4810      	ldr	r0, [pc, #64]	; (8003754 <SD_ReadByte+0x50>)
 8003714:	f04f 0102 	mov.w	r1, #2
 8003718:	f001 fb26 	bl	8004d68 <SPI_I2S_GetFlagStatus>
 800371c:	4603      	mov	r3, r0
 800371e:	2b00      	cmp	r3, #0
 8003720:	d0f7      	beq.n	8003712 <SD_ReadByte+0xe>
  {
  }
  /*!< Send the byte */
  SPI_I2S_SendData(SD_SPI, SD_DUMMY_BYTE);
 8003722:	480c      	ldr	r0, [pc, #48]	; (8003754 <SD_ReadByte+0x50>)
 8003724:	f04f 01ff 	mov.w	r1, #255	; 0xff
 8003728:	f001 fac3 	bl	8004cb2 <SPI_I2S_SendData>

  /*!< Wait until a data is received */
  while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
 800372c:	bf00      	nop
 800372e:	4809      	ldr	r0, [pc, #36]	; (8003754 <SD_ReadByte+0x50>)
 8003730:	f04f 0101 	mov.w	r1, #1
 8003734:	f001 fb18 	bl	8004d68 <SPI_I2S_GetFlagStatus>
 8003738:	4603      	mov	r3, r0
 800373a:	2b00      	cmp	r3, #0
 800373c:	d0f7      	beq.n	800372e <SD_ReadByte+0x2a>
  {
  }
  /*!< Get the received data */
  Data = SPI_I2S_ReceiveData(SD_SPI);
 800373e:	4805      	ldr	r0, [pc, #20]	; (8003754 <SD_ReadByte+0x50>)
 8003740:	f001 fab9 	bl	8004cb6 <SPI_I2S_ReceiveData>
 8003744:	4603      	mov	r3, r0
 8003746:	71fb      	strb	r3, [r7, #7]

  /*!< Return the shifted data */
  return Data;
 8003748:	79fb      	ldrb	r3, [r7, #7]
}
 800374a:	4618      	mov	r0, r3
 800374c:	f107 0708 	add.w	r7, r7, #8
 8003750:	46bd      	mov	sp, r7
 8003752:	bd80      	pop	{r7, pc}
 8003754:	40013000 	.word	0x40013000

08003758 <ADC_Configuration>:
#include "adc.h"

void ADC_Configuration(void)
{
 8003758:	b580      	push	{r7, lr}
 800375a:	b086      	sub	sp, #24
 800375c:	af00      	add	r7, sp, #0
  ADC_InitTypeDef  ADC_InitStructure;
  /* PCLK2 is the APB2 clock */
  /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
 800375e:	f44f 4000 	mov.w	r0, #32768	; 0x8000
 8003762:	f001 f8ab 	bl	80048bc <RCC_ADCCLKConfig>

  /* Enable ADC1 clock so that we can talk to it */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
 8003766:	f44f 7000 	mov.w	r0, #512	; 0x200
 800376a:	f04f 0101 	mov.w	r1, #1
 800376e:	f001 f931 	bl	80049d4 <RCC_APB2PeriphClockCmd>
  /* Put everything back to power-on defaults */
  ADC_DeInit(ADC1);
 8003772:	481c      	ldr	r0, [pc, #112]	; (80037e4 <ADC_Configuration+0x8c>)
 8003774:	f000 fc1c 	bl	8003fb0 <ADC_DeInit>

  /* ADC1 Configuration ------------------------------------------------------*/
  /* ADC1 and ADC2 operate independently */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
 8003778:	f04f 0300 	mov.w	r3, #0
 800377c:	607b      	str	r3, [r7, #4]
  /* Disable the scan conversion so we do one at a time */
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
 800377e:	f04f 0300 	mov.w	r3, #0
 8003782:	723b      	strb	r3, [r7, #8]
  /* Don't do contimuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
 8003784:	f04f 0300 	mov.w	r3, #0
 8003788:	727b      	strb	r3, [r7, #9]
  /* Start conversin by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
 800378a:	f44f 2360 	mov.w	r3, #917504	; 0xe0000
 800378e:	60fb      	str	r3, [r7, #12]
  /* Conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
 8003790:	f04f 0300 	mov.w	r3, #0
 8003794:	613b      	str	r3, [r7, #16]
  /* Say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfChannel = 1;
 8003796:	f04f 0301 	mov.w	r3, #1
 800379a:	753b      	strb	r3, [r7, #20]

  /* Now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);
 800379c:	f107 0304 	add.w	r3, r7, #4
 80037a0:	4810      	ldr	r0, [pc, #64]	; (80037e4 <ADC_Configuration+0x8c>)
 80037a2:	4619      	mov	r1, r3
 80037a4:	f000 fc32 	bl	800400c <ADC_Init>
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
 80037a8:	480e      	ldr	r0, [pc, #56]	; (80037e4 <ADC_Configuration+0x8c>)
 80037aa:	f04f 0101 	mov.w	r1, #1
 80037ae:	f000 fc5c 	bl	800406a <ADC_Cmd>

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
 80037b2:	480c      	ldr	r0, [pc, #48]	; (80037e4 <ADC_Configuration+0x8c>)
 80037b4:	f000 fc77 	bl	80040a6 <ADC_ResetCalibration>
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
 80037b8:	bf00      	nop
 80037ba:	480a      	ldr	r0, [pc, #40]	; (80037e4 <ADC_Configuration+0x8c>)
 80037bc:	f000 fc78 	bl	80040b0 <ADC_GetResetCalibrationStatus>
 80037c0:	4603      	mov	r3, r0
 80037c2:	2b00      	cmp	r3, #0
 80037c4:	d1f9      	bne.n	80037ba <ADC_Configuration+0x62>
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
 80037c6:	4807      	ldr	r0, [pc, #28]	; (80037e4 <ADC_Configuration+0x8c>)
 80037c8:	f000 fc76 	bl	80040b8 <ADC_StartCalibration>
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
 80037cc:	bf00      	nop
 80037ce:	4805      	ldr	r0, [pc, #20]	; (80037e4 <ADC_Configuration+0x8c>)
 80037d0:	f000 fc77 	bl	80040c2 <ADC_GetCalibrationStatus>
 80037d4:	4603      	mov	r3, r0
 80037d6:	2b00      	cmp	r3, #0
 80037d8:	d1f9      	bne.n	80037ce <ADC_Configuration+0x76>
}
 80037da:	f107 0718 	add.w	r7, r7, #24
 80037de:	46bd      	mov	sp, r7
 80037e0:	bd80      	pop	{r7, pc}
 80037e2:	bf00      	nop
 80037e4:	40012400 	.word	0x40012400

080037e8 <readADC1>:


uint16_t readADC1(uint8_t channel)
{
 80037e8:	b580      	push	{r7, lr}
 80037ea:	b082      	sub	sp, #8
 80037ec:	af00      	add	r7, sp, #0
 80037ee:	4603      	mov	r3, r0
 80037f0:	71fb      	strb	r3, [r7, #7]
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
 80037f2:	79fb      	ldrb	r3, [r7, #7]
 80037f4:	480f      	ldr	r0, [pc, #60]	; (8003834 <readADC1+0x4c>)
 80037f6:	4619      	mov	r1, r3
 80037f8:	f04f 0201 	mov.w	r2, #1
 80037fc:	f04f 0300 	mov.w	r3, #0
 8003800:	f000 fc83 	bl	800410a <ADC_RegularChannelConfig>
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
 8003804:	480b      	ldr	r0, [pc, #44]	; (8003834 <readADC1+0x4c>)
 8003806:	f04f 0101 	mov.w	r1, #1
 800380a:	f000 fc5e 	bl	80040ca <ADC_SoftwareStartConvCmd>
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
 800380e:	bf00      	nop
 8003810:	4808      	ldr	r0, [pc, #32]	; (8003834 <readADC1+0x4c>)
 8003812:	f04f 0102 	mov.w	r1, #2
 8003816:	f000 fd73 	bl	8004300 <ADC_GetFlagStatus>
 800381a:	4603      	mov	r3, r0
 800381c:	2b00      	cmp	r3, #0
 800381e:	d0f7      	beq.n	8003810 <readADC1+0x28>
  // Get the conversion value
  return ADC_GetConversionValue(ADC1);
 8003820:	4804      	ldr	r0, [pc, #16]	; (8003834 <readADC1+0x4c>)
 8003822:	f000 fcc3 	bl	80041ac <ADC_GetConversionValue>
 8003826:	4603      	mov	r3, r0
}
 8003828:	4618      	mov	r0, r3
 800382a:	f107 0708 	add.w	r7, r7, #8
 800382e:	46bd      	mov	sp, r7
 8003830:	bd80      	pop	{r7, pc}
 8003832:	bf00      	nop
 8003834:	40012400 	.word	0x40012400

08003838 <setup_gpio>:
#include "pwr.h"

uint8_t bootsource;

void setup_gpio(void)
{
 8003838:	b580      	push	{r7, lr}
 800383a:	b082      	sub	sp, #8
 800383c:	af00      	add	r7, sp, #0
	GPIO_InitTypeDef	GPIO_InitStructure;
	//enable the clocks 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);//GPIO clks
 800383e:	f04f 000c 	mov.w	r0, #12
 8003842:	f04f 0101 	mov.w	r1, #1
 8003846:	f001 f8c5 	bl	80049d4 <RCC_APB2PeriphClockCmd>
	setuppwr();				//configure power control
 800384a:	f000 fa89 	bl	8003d60 <setuppwr>
	disable_pin();				//disable WKUP pin functionality
 800384e:	f000 fa9d 	bl	8003d8c <disable_pin>
	//Configure and read the Charger EN pin - this has a pullup to V_USB, so if it reads 1 we booted off usb so setup USB detatch isr
	GPIO_InitStructure.GPIO_Pin = CHARGER_EN;
 8003852:	f04f 0304 	mov.w	r3, #4
 8003856:	80bb      	strh	r3, [r7, #4]
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
 8003858:	f04f 0328 	mov.w	r3, #40	; 0x28
 800385c:	71fb      	strb	r3, [r7, #7]
	GPIO_Init( GPIOB, &GPIO_InitStructure );/* configure pin 2 as input*/
 800385e:	f107 0304 	add.w	r3, r7, #4
 8003862:	482d      	ldr	r0, [pc, #180]	; (8003918 <setup_gpio+0xe0>)
 8003864:	4619      	mov	r1, r3
 8003866:	f000 fe48 	bl	80044fa <GPIO_Init>
	if(GPIO_ReadInputDataBit(GPIOB,CHARGER_EN))//We booted from USB
 800386a:	482b      	ldr	r0, [pc, #172]	; (8003918 <setup_gpio+0xe0>)
 800386c:	f04f 0104 	mov.w	r1, #4
 8003870:	f000 fe97 	bl	80045a2 <GPIO_ReadInputDataBit>
 8003874:	4603      	mov	r3, r0
 8003876:	2b00      	cmp	r3, #0
 8003878:	d003      	beq.n	8003882 <setup_gpio+0x4a>
		bootsource=USB_SOURCE;		//so we know
 800387a:	4b28      	ldr	r3, [pc, #160]	; (800391c <setup_gpio+0xe4>)
 800387c:	f04f 0201 	mov.w	r2, #1
 8003880:	701a      	strb	r2, [r3, #0]
	//Configure the io pins
	//LEDS + test
	GPIO_InitStructure.GPIO_Pin = RED|GREEN|TST;
 8003882:	f44f 53c8 	mov.w	r3, #6400	; 0x1900
 8003886:	80bb      	strh	r3, [r7, #4]
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 8003888:	f04f 0310 	mov.w	r3, #16
 800388c:	71fb      	strb	r3, [r7, #7]
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 800388e:	f04f 0303 	mov.w	r3, #3
 8003892:	71bb      	strb	r3, [r7, #6]
	GPIO_Init( GPIOB, &GPIO_InitStructure );/* configure pins 11 and 12 as output*/
 8003894:	f107 0304 	add.w	r3, r7, #4
 8003898:	481f      	ldr	r0, [pc, #124]	; (8003918 <setup_gpio+0xe0>)
 800389a:	4619      	mov	r1, r3
 800389c:	f000 fe2d 	bl	80044fa <GPIO_Init>
	//Power button
	GPIO_InitStructure.GPIO_Pin = WKUP;
 80038a0:	f04f 0301 	mov.w	r3, #1
 80038a4:	80bb      	strh	r3, [r7, #4]
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//pulldown
 80038a6:	f04f 0328 	mov.w	r3, #40	; 0x28
 80038aa:	71fb      	strb	r3, [r7, #7]
	GPIO_Init( GPIOA, &GPIO_InitStructure );/* configure WKUP pin as input pull up for button*/
 80038ac:	f107 0304 	add.w	r3, r7, #4
 80038b0:	481b      	ldr	r0, [pc, #108]	; (8003920 <setup_gpio+0xe8>)
 80038b2:	4619      	mov	r1, r3
 80038b4:	f000 fe21 	bl	80044fa <GPIO_Init>
	//Power supply enable
	GPIO_InitStructure.GPIO_Pin = PWREN;
 80038b8:	f44f 4300 	mov.w	r3, #32768	; 0x8000
 80038bc:	80bb      	strh	r3, [r7, #4]
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//pushpull
 80038be:	f04f 0310 	mov.w	r3, #16
 80038c2:	71fb      	strb	r3, [r7, #7]
	GPIO_Init( GPIOB, &GPIO_InitStructure );/* configure WKUP pin as input pull up for button*/
 80038c4:	f107 0304 	add.w	r3, r7, #4
 80038c8:	4813      	ldr	r0, [pc, #76]	; (8003918 <setup_gpio+0xe0>)
 80038ca:	4619      	mov	r1, r3
 80038cc:	f000 fe15 	bl	80044fa <GPIO_Init>
	GPIO_WriteBit(GPIOB,PWREN,Bit_SET);	//Make sure power enabled
 80038d0:	4811      	ldr	r0, [pc, #68]	; (8003918 <setup_gpio+0xe0>)
 80038d2:	f44f 4100 	mov.w	r1, #32768	; 0x8000
 80038d6:	f04f 0201 	mov.w	r2, #1
 80038da:	f000 fe78 	bl	80045ce <GPIO_WriteBit>
	//Configure the ADC inputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4;
 80038de:	f04f 0312 	mov.w	r3, #18
 80038e2:	80bb      	strh	r3, [r7, #4]
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 80038e4:	f04f 0300 	mov.w	r3, #0
 80038e8:	71fb      	strb	r3, [r7, #7]
	GPIO_Init( GPIOA, &GPIO_InitStructure );
 80038ea:	f107 0304 	add.w	r3, r7, #4
 80038ee:	480c      	ldr	r0, [pc, #48]	; (8003920 <setup_gpio+0xe8>)
 80038f0:	4619      	mov	r1, r3
 80038f2:	f000 fe02 	bl	80044fa <GPIO_Init>
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
 80038f6:	f04f 0303 	mov.w	r3, #3
 80038fa:	80bb      	strh	r3, [r7, #4]
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 80038fc:	f04f 0300 	mov.w	r3, #0
 8003900:	71fb      	strb	r3, [r7, #7]
	GPIO_Init( GPIOB, &GPIO_InitStructure );
 8003902:	f107 0304 	add.w	r3, r7, #4
 8003906:	4804      	ldr	r0, [pc, #16]	; (8003918 <setup_gpio+0xe0>)
 8003908:	4619      	mov	r1, r3
 800390a:	f000 fdf6 	bl	80044fa <GPIO_Init>
}
 800390e:	f107 0708 	add.w	r7, r7, #8
 8003912:	46bd      	mov	sp, r7
 8003914:	bd80      	pop	{r7, pc}
 8003916:	bf00      	nop
 8003918:	40010c00 	.word	0x40010c00
 800391c:	200004da 	.word	0x200004da
 8003920:	40010800 	.word	0x40010800

08003924 <switch_leds_on>:

void switch_leds_on(void)
{
 8003924:	b580      	push	{r7, lr}
 8003926:	af00      	add	r7, sp, #0
	if(USB_SOURCE==bootsource)
 8003928:	4b09      	ldr	r3, [pc, #36]	; (8003950 <switch_leds_on+0x2c>)
 800392a:	781b      	ldrb	r3, [r3, #0]
 800392c:	2b01      	cmp	r3, #1
 800392e:	d107      	bne.n	8003940 <switch_leds_on+0x1c>
		GPIO_WriteBit(GPIOB,RED,Bit_SET);
 8003930:	4808      	ldr	r0, [pc, #32]	; (8003954 <switch_leds_on+0x30>)
 8003932:	f44f 6100 	mov.w	r1, #2048	; 0x800
 8003936:	f04f 0201 	mov.w	r2, #1
 800393a:	f000 fe48 	bl	80045ce <GPIO_WriteBit>
 800393e:	e006      	b.n	800394e <switch_leds_on+0x2a>
	else
		GPIO_WriteBit(GPIOB,GREEN,Bit_SET);
 8003940:	4804      	ldr	r0, [pc, #16]	; (8003954 <switch_leds_on+0x30>)
 8003942:	f44f 5180 	mov.w	r1, #4096	; 0x1000
 8003946:	f04f 0201 	mov.w	r2, #1
 800394a:	f000 fe40 	bl	80045ce <GPIO_WriteBit>
}
 800394e:	bd80      	pop	{r7, pc}
 8003950:	200004da 	.word	0x200004da
 8003954:	40010c00 	.word	0x40010c00

08003958 <switch_leds_off>:

void switch_leds_off(void)
{
 8003958:	b580      	push	{r7, lr}
 800395a:	af00      	add	r7, sp, #0
	if(USB_SOURCE==bootsource)
 800395c:	4b09      	ldr	r3, [pc, #36]	; (8003984 <switch_leds_off+0x2c>)
 800395e:	781b      	ldrb	r3, [r3, #0]
 8003960:	2b01      	cmp	r3, #1
 8003962:	d107      	bne.n	8003974 <switch_leds_off+0x1c>
		GPIO_WriteBit(GPIOB,RED|TST,Bit_RESET);
 8003964:	4808      	ldr	r0, [pc, #32]	; (8003988 <switch_leds_off+0x30>)
 8003966:	f44f 6110 	mov.w	r1, #2304	; 0x900
 800396a:	f04f 0200 	mov.w	r2, #0
 800396e:	f000 fe2e 	bl	80045ce <GPIO_WriteBit>
 8003972:	e006      	b.n	8003982 <switch_leds_off+0x2a>
	else
		GPIO_WriteBit(GPIOB,GREEN|TST,Bit_RESET);
 8003974:	4804      	ldr	r0, [pc, #16]	; (8003988 <switch_leds_off+0x30>)
 8003976:	f44f 5188 	mov.w	r1, #4352	; 0x1100
 800397a:	f04f 0200 	mov.w	r2, #0
 800397e:	f000 fe26 	bl	80045ce <GPIO_WriteBit>
}
 8003982:	bd80      	pop	{r7, pc}
 8003984:	200004da 	.word	0x200004da
 8003988:	40010c00 	.word	0x40010c00

0800398c <power_off>:

void power_off()
{
 800398c:	b580      	push	{r7, lr}
 800398e:	b082      	sub	sp, #8
 8003990:	af00      	add	r7, sp, #0
	GPIO_InitTypeDef	GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
 8003992:	f04f 0304 	mov.w	r3, #4
 8003996:	71fb      	strb	r3, [r7, #7]
	GPIO_InitStructure.GPIO_Pin = 0xFFFF;	//all pins disabled on porta (WKUP setting overwrides gpio config)
 8003998:	f64f 73ff 	movw	r3, #65535	; 0xffff
 800399c:	80bb      	strh	r3, [r7, #4]
	GPIO_Init( GPIOA, &GPIO_InitStructure );
 800399e:	f107 0304 	add.w	r3, r7, #4
 80039a2:	480c      	ldr	r0, [pc, #48]	; (80039d4 <power_off+0x48>)
 80039a4:	4619      	mov	r1, r3
 80039a6:	f000 fda8 	bl	80044fa <GPIO_Init>
	GPIO_InitStructure.GPIO_Pin = ~PWREN;
 80039aa:	f647 73ff 	movw	r3, #32767	; 0x7fff
 80039ae:	80bb      	strh	r3, [r7, #4]
	GPIO_Init( GPIOB, &GPIO_InitStructure );//disable everything on port B apart from the power control pin, ready for shutdown	
 80039b0:	f107 0304 	add.w	r3, r7, #4
 80039b4:	4808      	ldr	r0, [pc, #32]	; (80039d8 <power_off+0x4c>)
 80039b6:	4619      	mov	r1, r3
 80039b8:	f000 fd9f 	bl	80044fa <GPIO_Init>
	GPIO_WriteBit(GPIOB,PWREN,Bit_RESET);	//power off
 80039bc:	4806      	ldr	r0, [pc, #24]	; (80039d8 <power_off+0x4c>)
 80039be:	f44f 4100 	mov.w	r1, #32768	; 0x8000
 80039c2:	f04f 0200 	mov.w	r2, #0
 80039c6:	f000 fe02 	bl	80045ce <GPIO_WriteBit>
}
 80039ca:	f107 0708 	add.w	r7, r7, #8
 80039ce:	46bd      	mov	sp, r7
 80039d0:	bd80      	pop	{r7, pc}
 80039d2:	bf00      	nop
 80039d4:	40010800 	.word	0x40010800
 80039d8:	40010c00 	.word	0x40010c00

080039dc <get_wkup>:

uint8_t get_wkup()
{
 80039dc:	b580      	push	{r7, lr}
 80039de:	af00      	add	r7, sp, #0
	return GPIO_ReadInputDataBit(GPIOA,WKUP);
 80039e0:	4803      	ldr	r0, [pc, #12]	; (80039f0 <get_wkup+0x14>)
 80039e2:	f04f 0101 	mov.w	r1, #1
 80039e6:	f000 fddc 	bl	80045a2 <GPIO_ReadInputDataBit>
 80039ea:	4603      	mov	r3, r0
}
 80039ec:	4618      	mov	r0, r3
 80039ee:	bd80      	pop	{r7, pc}
 80039f0:	40010800 	.word	0x40010800

080039f4 <EXTI_Config>:
  * @param  None
  * @retval None
  * Note that this is hardcoded to the ITG-3200 and LSM303DLH pins on v1.0 dactyl board
  * This initialiser function assumes the clocks and gpio have been configured
  */
void EXTI_Config(void) {
 80039f4:	b580      	push	{r7, lr}
 80039f6:	b084      	sub	sp, #16
 80039f8:	af00      	add	r7, sp, #0
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_DeInit();
 80039fa:	f000 fc99 	bl	8004330 <EXTI_DeInit>
	/* Connect EXTI0 Line to PA.0 pin - WKUP*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
 80039fe:	f04f 0000 	mov.w	r0, #0
 8003a02:	f04f 0100 	mov.w	r1, #0
 8003a06:	f000 fe33 	bl	8004670 <GPIO_EXTILineConfig>

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
 8003a0a:	f04f 0301 	mov.w	r3, #1
 8003a0e:	60bb      	str	r3, [r7, #8]
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
 8003a10:	f04f 0300 	mov.w	r3, #0
 8003a14:	733b      	strb	r3, [r7, #12]
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
 8003a16:	f04f 0308 	mov.w	r3, #8
 8003a1a:	737b      	strb	r3, [r7, #13]
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 8003a1c:	f04f 0301 	mov.w	r3, #1
 8003a20:	73bb      	strb	r3, [r7, #14]
	EXTI_Init(&EXTI_InitStructure);
 8003a22:	f107 0308 	add.w	r3, r7, #8
 8003a26:	4618      	mov	r0, r3
 8003a28:	f000 fc90 	bl	800434c <EXTI_Init>

	/* Set the Vector Table base location at 0x08000000 */    
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);      
 8003a2c:	f04f 6000 	mov.w	r0, #134217728	; 0x8000000
 8003a30:	f04f 0100 	mov.w	r1, #0
 8003a34:	f000 fa9a 	bl	8003f6c <NVIC_SetVectorTable>
	//First we configure the Kalman ISR
	/* Configure one bit for preemption priority */   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
 8003a38:	f44f 60c0 	mov.w	r0, #1536	; 0x600
 8003a3c:	f000 fa5c 	bl	8003ef8 <NVIC_PriorityGroupConfig>
	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	//The WKUP triggered interrupt	
 8003a40:	f04f 0306 	mov.w	r3, #6
 8003a44:	713b      	strb	r3, [r7, #4]
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority
 8003a46:	f04f 0301 	mov.w	r3, #1
 8003a4a:	717b      	strb	r3, [r7, #5]
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;	//low group priority
 8003a4c:	f04f 0307 	mov.w	r3, #7
 8003a50:	71bb      	strb	r3, [r7, #6]
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 8003a52:	f04f 0301 	mov.w	r3, #1
 8003a56:	71fb      	strb	r3, [r7, #7]
	NVIC_Init(&NVIC_InitStructure);
 8003a58:	f107 0304 	add.w	r3, r7, #4
 8003a5c:	4618      	mov	r0, r3
 8003a5e:	f000 fa55 	bl	8003f0c <NVIC_Init>
	if(USB_SOURCE==bootsource) {				//we booted from USB - configure an interrupt on USB removal
 8003a62:	4b18      	ldr	r3, [pc, #96]	; (8003ac4 <EXTI_Config+0xd0>)
 8003a64:	781b      	ldrb	r3, [r3, #0]
 8003a66:	2b01      	cmp	r3, #1
 8003a68:	d127      	bne.n	8003aba <EXTI_Config+0xc6>
		/* Connect EXTI2 Line to PB.2 pin - CHARGER_EN*/
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
 8003a6a:	f04f 0001 	mov.w	r0, #1
 8003a6e:	f04f 0102 	mov.w	r1, #2
 8003a72:	f000 fdfd 	bl	8004670 <GPIO_EXTILineConfig>
		/* Configure EXTI2 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line2;
 8003a76:	f04f 0304 	mov.w	r3, #4
 8003a7a:	60bb      	str	r3, [r7, #8]
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
 8003a7c:	f04f 0300 	mov.w	r3, #0
 8003a80:	733b      	strb	r3, [r7, #12]
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
 8003a82:	f04f 030c 	mov.w	r3, #12
 8003a86:	737b      	strb	r3, [r7, #13]
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 8003a88:	f04f 0301 	mov.w	r3, #1
 8003a8c:	73bb      	strb	r3, [r7, #14]
		EXTI_Init(&EXTI_InitStructure);
 8003a8e:	f107 0308 	add.w	r3, r7, #8
 8003a92:	4618      	mov	r0, r3
 8003a94:	f000 fc5a 	bl	800434c <EXTI_Init>
		/* Enable and set EXTI2 Interrupt to the second lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//The WKUP triggered interrupt	
 8003a98:	f04f 0308 	mov.w	r3, #8
 8003a9c:	713b      	strb	r3, [r7, #4]
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority
 8003a9e:	f04f 0301 	mov.w	r3, #1
 8003aa2:	717b      	strb	r3, [r7, #5]
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;//low group priority
 8003aa4:	f04f 0306 	mov.w	r3, #6
 8003aa8:	71bb      	strb	r3, [r7, #6]
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 8003aaa:	f04f 0301 	mov.w	r3, #1
 8003aae:	71fb      	strb	r3, [r7, #7]
		NVIC_Init(&NVIC_InitStructure);
 8003ab0:	f107 0304 	add.w	r3, r7, #4
 8003ab4:	4618      	mov	r0, r3
 8003ab6:	f000 fa29 	bl	8003f0c <NVIC_Init>
	}
}
 8003aba:	f107 0710 	add.w	r7, r7, #16
 8003abe:	46bd      	mov	sp, r7
 8003ac0:	bd80      	pop	{r7, pc}
 8003ac2:	bf00      	nop
 8003ac4:	200004da 	.word	0x200004da

08003ac8 <EXTI0_IRQHandler>:
/**
  * @brief  This function handles External line 0 interrupt request.- WKUP ISR
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void) {
 8003ac8:	b580      	push	{r7, lr}
 8003aca:	af00      	add	r7, sp, #0
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
 8003acc:	f04f 0001 	mov.w	r0, #1
 8003ad0:	f000 fc9e 	bl	8004410 <EXTI_GetITStatus>
 8003ad4:	4603      	mov	r3, r0
 8003ad6:	2b00      	cmp	r3, #0
 8003ad8:	d005      	beq.n	8003ae6 <EXTI0_IRQHandler+0x1e>
		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
 8003ada:	f04f 0001 	mov.w	r0, #1
 8003ade:	f000 fca5 	bl	800442c <EXTI_ClearITPendingBit>
		/*Called Code goes here*/
		//power_off();					//disables the gpio
		shutdown();					//shuts down - only wakes up on power pin i.e. WKUP
 8003ae2:	f000 f949 	bl	8003d78 <shutdown>
	}
}
 8003ae6:	bd80      	pop	{r7, pc}

08003ae8 <EXTI2_IRQHandler>:
/**
  * @brief  This function handles External line 2 interrupt request.- CHARGER_EN (USB unplugged) ISR
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void) {
 8003ae8:	b580      	push	{r7, lr}
 8003aea:	af00      	add	r7, sp, #0
	if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
 8003aec:	f04f 0004 	mov.w	r0, #4
 8003af0:	f000 fc8e 	bl	8004410 <EXTI_GetITStatus>
 8003af4:	4603      	mov	r3, r0
 8003af6:	2b00      	cmp	r3, #0
 8003af8:	d005      	beq.n	8003b06 <EXTI2_IRQHandler+0x1e>
		/* Clear the  EXTI line 2 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);
 8003afa:	f04f 0004 	mov.w	r0, #4
 8003afe:	f000 fc95 	bl	800442c <EXTI_ClearITPendingBit>
		/*Called Code goes here*/
		//power_off();					//disables the gpio
		shutdown();					//shuts down - only wakes up on power pin i.e. WKUP
 8003b02:	f000 f939 	bl	8003d78 <shutdown>
	}
}
 8003b06:	bd80      	pop	{r7, pc}

08003b08 <USB_HP_CAN_TX_IRQHandler>:
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{
 8003b08:	b580      	push	{r7, lr}
 8003b0a:	af00      	add	r7, sp, #0
  CTR_HP();
 8003b0c:	f002 f934 	bl	8005d78 <CTR_HP>
}
 8003b10:	bd80      	pop	{r7, pc}
 8003b12:	bf00      	nop

08003b14 <USB_LP_CAN_RX0_IRQHandler>:
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
 8003b14:	b580      	push	{r7, lr}
 8003b16:	af00      	add	r7, sp, #0
  USB_Istr();
 8003b18:	f7fd faf4 	bl	8001104 <USB_Istr>
}
 8003b1c:	bd80      	pop	{r7, pc}
 8003b1e:	bf00      	nop

08003b20 <ADC1_2_IRQHandler>:
 * @retval : None       
*/
    .section	.text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
 8003b20:	e7fe      	b.n	8003b20 <ADC1_2_IRQHandler>
	...

08003b24 <SystemInit>:
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
void SystemInit (void)
{
 8003b24:	b580      	push	{r7, lr}
 8003b26:	af00      	add	r7, sp, #0
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;
 8003b28:	4b13      	ldr	r3, [pc, #76]	; (8003b78 <SystemInit+0x54>)
 8003b2a:	4a13      	ldr	r2, [pc, #76]	; (8003b78 <SystemInit+0x54>)
 8003b2c:	6812      	ldr	r2, [r2, #0]
 8003b2e:	f042 0201 	orr.w	r2, r2, #1
 8003b32:	601a      	str	r2, [r3, #0]

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
#ifndef STM32F10X_CL
  RCC->CFGR &= (uint32_t)0xF8FF0000;
 8003b34:	4a10      	ldr	r2, [pc, #64]	; (8003b78 <SystemInit+0x54>)
 8003b36:	4b10      	ldr	r3, [pc, #64]	; (8003b78 <SystemInit+0x54>)
 8003b38:	6859      	ldr	r1, [r3, #4]
 8003b3a:	4b10      	ldr	r3, [pc, #64]	; (8003b7c <SystemInit+0x58>)
 8003b3c:	ea01 0303 	and.w	r3, r1, r3
 8003b40:	6053      	str	r3, [r2, #4]
#else
  RCC->CFGR &= (uint32_t)0xF0FF0000;
#endif /* STM32F10X_CL */   
  
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;
 8003b42:	4a0d      	ldr	r2, [pc, #52]	; (8003b78 <SystemInit+0x54>)
 8003b44:	4b0c      	ldr	r3, [pc, #48]	; (8003b78 <SystemInit+0x54>)
 8003b46:	681b      	ldr	r3, [r3, #0]
 8003b48:	f023 7384 	bic.w	r3, r3, #17301504	; 0x1080000
 8003b4c:	f423 3380 	bic.w	r3, r3, #65536	; 0x10000
 8003b50:	6013      	str	r3, [r2, #0]

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;
 8003b52:	4b09      	ldr	r3, [pc, #36]	; (8003b78 <SystemInit+0x54>)
 8003b54:	4a08      	ldr	r2, [pc, #32]	; (8003b78 <SystemInit+0x54>)
 8003b56:	6812      	ldr	r2, [r2, #0]
 8003b58:	f422 2280 	bic.w	r2, r2, #262144	; 0x40000
 8003b5c:	601a      	str	r2, [r3, #0]

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;
 8003b5e:	4b06      	ldr	r3, [pc, #24]	; (8003b78 <SystemInit+0x54>)
 8003b60:	4a05      	ldr	r2, [pc, #20]	; (8003b78 <SystemInit+0x54>)
 8003b62:	6852      	ldr	r2, [r2, #4]
 8003b64:	f422 02fe 	bic.w	r2, r2, #8323072	; 0x7f0000
 8003b68:	605a      	str	r2, [r3, #4]

#ifndef STM32F10X_CL
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;
 8003b6a:	4b03      	ldr	r3, [pc, #12]	; (8003b78 <SystemInit+0x54>)
 8003b6c:	f44f 021f 	mov.w	r2, #10420224	; 0x9f0000
 8003b70:	609a      	str	r2, [r3, #8]
  RCC->CFGR2 = 0x00000000;
#endif /* STM32F10X_CL */
    
  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */
  SetSysClock();
 8003b72:	f000 f805 	bl	8003b80 <SetSysClock>

}
 8003b76:	bd80      	pop	{r7, pc}
 8003b78:	40021000 	.word	0x40021000
 8003b7c:	f8ff0000 	.word	0xf8ff0000

08003b80 <SetSysClock>:
  * @brief  Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers.
  * @param  None
  * @retval None
  */
static void SetSysClock(void)
{
 8003b80:	b580      	push	{r7, lr}
 8003b82:	af00      	add	r7, sp, #0
#elif defined SYSCLK_FREQ_48MHz
  SetSysClockTo48();
#elif defined SYSCLK_FREQ_56MHz
  SetSysClockTo56();  
#elif defined SYSCLK_FREQ_72MHz
  SetSysClockTo72();
 8003b84:	f000 f802 	bl	8003b8c <SetSysClockTo72>
#endif
 
 /* If none of the define above is enabled, the HSI is used as System clock
    source (default after reset) */ 
}
 8003b88:	bd80      	pop	{r7, pc}
 8003b8a:	bf00      	nop

08003b8c <SetSysClockTo72>:
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo72(void)
{
 8003b8c:	b480      	push	{r7}
 8003b8e:	b083      	sub	sp, #12
 8003b90:	af00      	add	r7, sp, #0
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
 8003b92:	f04f 0300 	mov.w	r3, #0
 8003b96:	607b      	str	r3, [r7, #4]
 8003b98:	f04f 0300 	mov.w	r3, #0
 8003b9c:	603b      	str	r3, [r7, #0]
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 8003b9e:	4b3c      	ldr	r3, [pc, #240]	; (8003c90 <SetSysClockTo72+0x104>)
 8003ba0:	4a3b      	ldr	r2, [pc, #236]	; (8003c90 <SetSysClockTo72+0x104>)
 8003ba2:	6812      	ldr	r2, [r2, #0]
 8003ba4:	f442 3280 	orr.w	r2, r2, #65536	; 0x10000
 8003ba8:	601a      	str	r2, [r3, #0]
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
 8003baa:	4b39      	ldr	r3, [pc, #228]	; (8003c90 <SetSysClockTo72+0x104>)
 8003bac:	681b      	ldr	r3, [r3, #0]
 8003bae:	f403 3300 	and.w	r3, r3, #131072	; 0x20000
 8003bb2:	603b      	str	r3, [r7, #0]
    StartUpCounter++;  
 8003bb4:	687b      	ldr	r3, [r7, #4]
 8003bb6:	f103 0301 	add.w	r3, r3, #1
 8003bba:	607b      	str	r3, [r7, #4]
  } while((HSEStatus == 0) && (StartUpCounter != HSEStartUp_TimeOut));
 8003bbc:	683b      	ldr	r3, [r7, #0]
 8003bbe:	2b00      	cmp	r3, #0
 8003bc0:	d103      	bne.n	8003bca <SetSysClockTo72+0x3e>
 8003bc2:	687b      	ldr	r3, [r7, #4]
 8003bc4:	f5b3 6fa0 	cmp.w	r3, #1280	; 0x500
 8003bc8:	d1ef      	bne.n	8003baa <SetSysClockTo72+0x1e>

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
 8003bca:	4b31      	ldr	r3, [pc, #196]	; (8003c90 <SetSysClockTo72+0x104>)
 8003bcc:	681b      	ldr	r3, [r3, #0]
 8003bce:	f403 3300 	and.w	r3, r3, #131072	; 0x20000
 8003bd2:	2b00      	cmp	r3, #0
 8003bd4:	d003      	beq.n	8003bde <SetSysClockTo72+0x52>
  {
    HSEStatus = (uint32_t)0x01;
 8003bd6:	f04f 0301 	mov.w	r3, #1
 8003bda:	603b      	str	r3, [r7, #0]
 8003bdc:	e002      	b.n	8003be4 <SetSysClockTo72+0x58>
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
 8003bde:	f04f 0300 	mov.w	r3, #0
 8003be2:	603b      	str	r3, [r7, #0]
  }  

  if (HSEStatus == (uint32_t)0x01)
 8003be4:	683b      	ldr	r3, [r7, #0]
 8003be6:	2b01      	cmp	r3, #1
 8003be8:	d14c      	bne.n	8003c84 <SetSysClockTo72+0xf8>
  {
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
 8003bea:	4b2a      	ldr	r3, [pc, #168]	; (8003c94 <SetSysClockTo72+0x108>)
 8003bec:	4a29      	ldr	r2, [pc, #164]	; (8003c94 <SetSysClockTo72+0x108>)
 8003bee:	6812      	ldr	r2, [r2, #0]
 8003bf0:	f042 0210 	orr.w	r2, r2, #16
 8003bf4:	601a      	str	r2, [r3, #0]

    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
 8003bf6:	4b27      	ldr	r3, [pc, #156]	; (8003c94 <SetSysClockTo72+0x108>)
 8003bf8:	4a26      	ldr	r2, [pc, #152]	; (8003c94 <SetSysClockTo72+0x108>)
 8003bfa:	6812      	ldr	r2, [r2, #0]
 8003bfc:	f022 0203 	bic.w	r2, r2, #3
 8003c00:	601a      	str	r2, [r3, #0]
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    
 8003c02:	4b24      	ldr	r3, [pc, #144]	; (8003c94 <SetSysClockTo72+0x108>)
 8003c04:	4a23      	ldr	r2, [pc, #140]	; (8003c94 <SetSysClockTo72+0x108>)
 8003c06:	6812      	ldr	r2, [r2, #0]
 8003c08:	f042 0202 	orr.w	r2, r2, #2
 8003c0c:	601a      	str	r2, [r3, #0]

 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
 8003c0e:	4b20      	ldr	r3, [pc, #128]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c10:	4a1f      	ldr	r2, [pc, #124]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c12:	6852      	ldr	r2, [r2, #4]
 8003c14:	605a      	str	r2, [r3, #4]
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
 8003c16:	4b1e      	ldr	r3, [pc, #120]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c18:	4a1d      	ldr	r2, [pc, #116]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c1a:	6852      	ldr	r2, [r2, #4]
 8003c1c:	605a      	str	r2, [r3, #4]
    
    /* PCLK1 = HCLK/2 */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
 8003c1e:	4b1c      	ldr	r3, [pc, #112]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c20:	4a1b      	ldr	r2, [pc, #108]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c22:	6852      	ldr	r2, [r2, #4]
 8003c24:	f442 6280 	orr.w	r2, r2, #1024	; 0x400
 8003c28:	605a      	str	r2, [r3, #4]
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | 
                            RCC_CFGR_PLLMULL9); 
#else /* note changed for 12mhz external xtal */   
    /*  PLL configuration: PLLCLK = HSE * 6 = 72 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
 8003c2a:	4b19      	ldr	r3, [pc, #100]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c2c:	4a18      	ldr	r2, [pc, #96]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c2e:	6852      	ldr	r2, [r2, #4]
 8003c30:	f422 127c 	bic.w	r2, r2, #4128768	; 0x3f0000
 8003c34:	605a      	str	r2, [r3, #4]
                                        RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL6);
 8003c36:	4b16      	ldr	r3, [pc, #88]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c38:	4a15      	ldr	r2, [pc, #84]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c3a:	6852      	ldr	r2, [r2, #4]
 8003c3c:	f442 1288 	orr.w	r2, r2, #1114112	; 0x110000
 8003c40:	605a      	str	r2, [r3, #4]
#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
 8003c42:	4b13      	ldr	r3, [pc, #76]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c44:	4a12      	ldr	r2, [pc, #72]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c46:	6812      	ldr	r2, [r2, #0]
 8003c48:	f042 7280 	orr.w	r2, r2, #16777216	; 0x1000000
 8003c4c:	601a      	str	r2, [r3, #0]

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
 8003c4e:	bf00      	nop
 8003c50:	4b0f      	ldr	r3, [pc, #60]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c52:	681b      	ldr	r3, [r3, #0]
 8003c54:	f003 7300 	and.w	r3, r3, #33554432	; 0x2000000
 8003c58:	2b00      	cmp	r3, #0
 8003c5a:	d0f9      	beq.n	8003c50 <SetSysClockTo72+0xc4>
    {
    }
    
    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
 8003c5c:	4b0c      	ldr	r3, [pc, #48]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c5e:	4a0c      	ldr	r2, [pc, #48]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c60:	6852      	ldr	r2, [r2, #4]
 8003c62:	f022 0203 	bic.w	r2, r2, #3
 8003c66:	605a      	str	r2, [r3, #4]
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    
 8003c68:	4b09      	ldr	r3, [pc, #36]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c6a:	4a09      	ldr	r2, [pc, #36]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c6c:	6852      	ldr	r2, [r2, #4]
 8003c6e:	f042 0202 	orr.w	r2, r2, #2
 8003c72:	605a      	str	r2, [r3, #4]

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
 8003c74:	bf00      	nop
 8003c76:	4b06      	ldr	r3, [pc, #24]	; (8003c90 <SetSysClockTo72+0x104>)
 8003c78:	685b      	ldr	r3, [r3, #4]
 8003c7a:	f003 030c 	and.w	r3, r3, #12
 8003c7e:	2b08      	cmp	r3, #8
 8003c80:	d1f9      	bne.n	8003c76 <SetSysClockTo72+0xea>
 8003c82:	e000      	b.n	8003c86 <SetSysClockTo72+0xfa>
         configuration. User can add here some code to deal with this error */    

    /* Go to infinite loop */
    while (1)
    {
    }
 8003c84:	e7fe      	b.n	8003c84 <SetSysClockTo72+0xf8>
  }
}
 8003c86:	f107 070c 	add.w	r7, r7, #12
 8003c8a:	46bd      	mov	sp, r7
 8003c8c:	bc80      	pop	{r7}
 8003c8e:	4770      	bx	lr
 8003c90:	40021000 	.word	0x40021000
 8003c94:	40022000 	.word	0x40022000

08003c98 <main>:
} while (0)

extern uint16_t MAL_Init (uint8_t lun);			//For the USB filesystem driver

int main(void)
{
 8003c98:	b598      	push	{r3, r4, r7, lr}
 8003c9a:	af00      	add	r7, sp, #0
	SystemInit();
 8003c9c:	f7ff ff42 	bl	8003b24 <SystemInit>
	setup_gpio();
 8003ca0:	f7ff fdca 	bl	8003838 <setup_gpio>
	ADC_Configuration();
 8003ca4:	f7ff fd58 	bl	8003758 <ADC_Configuration>
	Usarts_Init();
 8003ca8:	f000 f878 	bl	8003d9c <Usarts_Init>
	calibrate_sensor();
 8003cac:	f7fc fa38 	bl	8000120 <calibrate_sensor>
	EXTI_Config();
 8003cb0:	f7ff fea0 	bl	80039f4 <EXTI_Config>
	rprintfInit(__usart_send_char);
 8003cb4:	4824      	ldr	r0, [pc, #144]	; (8003d48 <main+0xb0>)
 8003cb6:	f7fe f8c7 	bl	8001e48 <rprintfInit>
	if(USB_SOURCE==bootsource) {
 8003cba:	4b24      	ldr	r3, [pc, #144]	; (8003d4c <main+0xb4>)
 8003cbc:	781b      	ldrb	r3, [r3, #0]
 8003cbe:	2b01      	cmp	r3, #1
 8003cc0:	d112      	bne.n	8003ce8 <main+0x50>
		Set_System();				//This actually just inits the storage layer
 8003cc2:	f7fc faad 	bl	8000220 <Set_System>
		Set_USBClock();
 8003cc6:	f7fc fab1 	bl	800022c <Set_USBClock>
		USB_Interrupts_Config();
 8003cca:	f7fc fae5 	bl	8000298 <USB_Interrupts_Config>
		USB_Init();
 8003cce:	f001 ff53 	bl	8005b78 <USB_Init>
		while (bDeviceState != CONFIGURED);	//Wait for USB config
 8003cd2:	bf00      	nop
 8003cd4:	4b1e      	ldr	r3, [pc, #120]	; (8003d50 <main+0xb8>)
 8003cd6:	681b      	ldr	r3, [r3, #0]
 8003cd8:	2b05      	cmp	r3, #5
 8003cda:	d1fb      	bne.n	8003cd4 <main+0x3c>
		USB_Configured_LED();
 8003cdc:	f7fc fb22 	bl	8000324 <USB_Configured_LED>
 8003ce0:	e002      	b.n	8003ce8 <main+0x50>
		switch_leds_off();
		delay();
		printf("Pressure:%f\r\n",conv_adc_diff());
		if(!GPIO_ReadInputDataBit(GPIOB,CHARGER_EN) && USB_SOURCE==bootsource)	//We booted from USB, and USB removed (EXTI2 wont work?!)
			shutdown();
	}
 8003ce2:	bf00      	nop
 8003ce4:	e000      	b.n	8003ce8 <main+0x50>
 8003ce6:	bf00      	nop
		USB_Init();
		while (bDeviceState != CONFIGURED);	//Wait for USB config
		USB_Configured_LED();
	}
	while (1) {
		switch_leds_on();
 8003ce8:	f7ff fe1c 	bl	8003924 <switch_leds_on>
		delay();
 8003cec:	f04f 0400 	mov.w	r4, #0
 8003cf0:	e002      	b.n	8003cf8 <main+0x60>
 8003cf2:	bf00      	nop
 8003cf4:	f104 0401 	add.w	r4, r4, #1
 8003cf8:	4b16      	ldr	r3, [pc, #88]	; (8003d54 <main+0xbc>)
 8003cfa:	429c      	cmp	r4, r3
 8003cfc:	d9f9      	bls.n	8003cf2 <main+0x5a>
		switch_leds_off();
 8003cfe:	f7ff fe2b 	bl	8003958 <switch_leds_off>
		delay();
 8003d02:	f04f 0400 	mov.w	r4, #0
 8003d06:	e002      	b.n	8003d0e <main+0x76>
 8003d08:	bf00      	nop
 8003d0a:	f104 0401 	add.w	r4, r4, #1
 8003d0e:	4b11      	ldr	r3, [pc, #68]	; (8003d54 <main+0xbc>)
 8003d10:	429c      	cmp	r4, r3
 8003d12:	d9f9      	bls.n	8003d08 <main+0x70>
		printf("Pressure:%f\r\n",conv_adc_diff());
 8003d14:	f7fc fa44 	bl	80001a0 <conv_adc_diff>
 8003d18:	4603      	mov	r3, r0
 8003d1a:	4618      	mov	r0, r3
 8003d1c:	f002 feac 	bl	8006a78 <__aeabi_f2d>
 8003d20:	4602      	mov	r2, r0
 8003d22:	460b      	mov	r3, r1
 8003d24:	480c      	ldr	r0, [pc, #48]	; (8003d58 <main+0xc0>)
 8003d26:	f7fe fadd 	bl	80022e4 <rprintf2RamRom>
		if(!GPIO_ReadInputDataBit(GPIOB,CHARGER_EN) && USB_SOURCE==bootsource)	//We booted from USB, and USB removed (EXTI2 wont work?!)
 8003d2a:	480c      	ldr	r0, [pc, #48]	; (8003d5c <main+0xc4>)
 8003d2c:	f04f 0104 	mov.w	r1, #4
 8003d30:	f000 fc37 	bl	80045a2 <GPIO_ReadInputDataBit>
 8003d34:	4603      	mov	r3, r0
 8003d36:	2b00      	cmp	r3, #0
 8003d38:	d1d3      	bne.n	8003ce2 <main+0x4a>
 8003d3a:	4b04      	ldr	r3, [pc, #16]	; (8003d4c <main+0xb4>)
 8003d3c:	781b      	ldrb	r3, [r3, #0]
 8003d3e:	2b01      	cmp	r3, #1
 8003d40:	d1d1      	bne.n	8003ce6 <main+0x4e>
			shutdown();
 8003d42:	f000 f819 	bl	8003d78 <shutdown>
	}
 8003d46:	e7cf      	b.n	8003ce8 <main+0x50>
 8003d48:	08003ec5 	.word	0x08003ec5
 8003d4c:	200004da 	.word	0x200004da
 8003d50:	200001ac 	.word	0x200001ac
 8003d54:	000f423f 	.word	0x000f423f
 8003d58:	08007764 	.word	0x08007764
 8003d5c:	40010c00 	.word	0x40010c00

08003d60 <setuppwr>:
#include "pwr.h"

void setuppwr() {
 8003d60:	b580      	push	{r7, lr}
 8003d62:	af00      	add	r7, sp, #0
	PWR_DeInit();
 8003d64:	f000 fca6 	bl	80046b4 <PWR_DeInit>
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//clk to the pwr control
 8003d68:	f04f 5080 	mov.w	r0, #268435456	; 0x10000000
 8003d6c:	f04f 0101 	mov.w	r1, #1
 8003d70:	f000 fe3c 	bl	80049ec <RCC_APB1PeriphClockCmd>
}
 8003d74:	bd80      	pop	{r7, pc}
 8003d76:	bf00      	nop

08003d78 <shutdown>:

void shutdown() {
 8003d78:	b580      	push	{r7, lr}
 8003d7a:	af00      	add	r7, sp, #0
	PWR_WakeUpPinCmd(ENABLE);			//enable the pin
 8003d7c:	f04f 0001 	mov.w	r0, #1
 8003d80:	f000 fcba 	bl	80046f8 <PWR_WakeUpPinCmd>
	PWR_EnterSTANDBYMode();				//only wakes on RTC signals or WKUP pin
 8003d84:	f000 fcd4 	bl	8004730 <PWR_EnterSTANDBYMode>
}
 8003d88:	bd80      	pop	{r7, pc}
 8003d8a:	bf00      	nop

08003d8c <disable_pin>:

void disable_pin() {
 8003d8c:	b580      	push	{r7, lr}
 8003d8e:	af00      	add	r7, sp, #0
	PWR_WakeUpPinCmd(DISABLE);			//disable the pin
 8003d90:	f04f 0000 	mov.w	r0, #0
 8003d94:	f000 fcb0 	bl	80046f8 <PWR_WakeUpPinCmd>
}
 8003d98:	bd80      	pop	{r7, pc}
 8003d9a:	bf00      	nop

08003d9c <Usarts_Init>:
/**
  * @brief  Configured the USART1 and 2 periferals, including clocks
  * @param  None
  * @retval None
  */
void Usarts_Init() {
 8003d9c:	b580      	push	{r7, lr}
 8003d9e:	b086      	sub	sp, #24
 8003da0:	af00      	add	r7, sp, #0
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    
    // Enable clock to GPIO and USART1 and USART2 peripherals - on different APBs
    RCC_APB2PeriphClockCmd(USART1_RCC_GPIO | USART1_RCC_USART, ENABLE);
 8003da2:	f244 0004 	movw	r0, #16388	; 0x4004
 8003da6:	f04f 0101 	mov.w	r1, #1
 8003daa:	f000 fe13 	bl	80049d4 <RCC_APB2PeriphClockCmd>
    RCC_APB1PeriphClockCmd(USART2_RCC_USART,ENABLE );
 8003dae:	f44f 3000 	mov.w	r0, #131072	; 0x20000
 8003db2:	f04f 0101 	mov.w	r1, #1
 8003db6:	f000 fe19 	bl	80049ec <RCC_APB1PeriphClockCmd>

    // Configure Tx pins
    GPIO_InitStructure.GPIO_Pin     = USART1_TX | USART2_TX;
 8003dba:	f44f 7301 	mov.w	r3, #516	; 0x204
 8003dbe:	82bb      	strh	r3, [r7, #20]
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
 8003dc0:	f04f 0303 	mov.w	r3, #3
 8003dc4:	75bb      	strb	r3, [r7, #22]
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
 8003dc6:	f04f 0318 	mov.w	r3, #24
 8003dca:	75fb      	strb	r3, [r7, #23]
    GPIO_Init(USART_GPIO, &GPIO_InitStructure);
 8003dcc:	f107 0314 	add.w	r3, r7, #20
 8003dd0:	481b      	ldr	r0, [pc, #108]	; (8003e40 <Usarts_Init+0xa4>)
 8003dd2:	4619      	mov	r1, r3
 8003dd4:	f000 fb91 	bl	80044fa <GPIO_Init>
    
    // Configure Rx pins
    GPIO_InitStructure.GPIO_Pin     = USART1_RX | USART2_RX;
 8003dd8:	f44f 6381 	mov.w	r3, #1032	; 0x408
 8003ddc:	82bb      	strh	r3, [r7, #20]
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
 8003dde:	f04f 0303 	mov.w	r3, #3
 8003de2:	75bb      	strb	r3, [r7, #22]
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN_FLOATING;
 8003de4:	f04f 0304 	mov.w	r3, #4
 8003de8:	75fb      	strb	r3, [r7, #23]
    GPIO_Init(USART_GPIO, &GPIO_InitStructure);
 8003dea:	f107 0314 	add.w	r3, r7, #20
 8003dee:	4814      	ldr	r0, [pc, #80]	; (8003e40 <Usarts_Init+0xa4>)
 8003df0:	4619      	mov	r1, r3
 8003df2:	f000 fb82 	bl	80044fa <GPIO_Init>
    
    // Configure USART1 peripheral
    USART_InitStructure.USART_BaudRate  = USART1_BAUD;
 8003df6:	f44f 33e1 	mov.w	r3, #115200	; 0x1c200
 8003dfa:	607b      	str	r3, [r7, #4]
    Default_Usart_Config(&USART_InitStructure);
 8003dfc:	f107 0304 	add.w	r3, r7, #4
 8003e00:	4618      	mov	r0, r3
 8003e02:	f000 f823 	bl	8003e4c <Default_Usart_Config>
    USART_Init(USART1_USART, &USART_InitStructure );
 8003e06:	f107 0304 	add.w	r3, r7, #4
 8003e0a:	480e      	ldr	r0, [pc, #56]	; (8003e44 <Usarts_Init+0xa8>)
 8003e0c:	4619      	mov	r1, r3
 8003e0e:	f001 f81b 	bl	8004e48 <USART_Init>
    // Configure USART2 peripheral - only buadrate is changed
    USART_InitStructure.USART_BaudRate = USART2_BAUD;
 8003e12:	f44f 33e1 	mov.w	r3, #115200	; 0x1c200
 8003e16:	607b      	str	r3, [r7, #4]
    USART_Init(USART2_USART, &USART_InitStructure );
 8003e18:	f107 0304 	add.w	r3, r7, #4
 8003e1c:	480a      	ldr	r0, [pc, #40]	; (8003e48 <Usarts_Init+0xac>)
 8003e1e:	4619      	mov	r1, r3
 8003e20:	f001 f812 	bl	8004e48 <USART_Init>

    /* Enable USART2 DMA Rx request */
    //USART_DMACmd(USART2_USART, USART_DMAReq_Rx , ENABLE);

    /* Enable the USART1 */
    USART_Cmd(USART1_USART, ENABLE);
 8003e24:	4807      	ldr	r0, [pc, #28]	; (8003e44 <Usarts_Init+0xa8>)
 8003e26:	f04f 0101 	mov.w	r1, #1
 8003e2a:	f001 f879 	bl	8004f20 <USART_Cmd>
    /* Enable the USART2 */
    USART_Cmd(USART2_USART, ENABLE);
 8003e2e:	4806      	ldr	r0, [pc, #24]	; (8003e48 <Usarts_Init+0xac>)
 8003e30:	f04f 0101 	mov.w	r1, #1
 8003e34:	f001 f874 	bl	8004f20 <USART_Cmd>
}
 8003e38:	f107 0718 	add.w	r7, r7, #24
 8003e3c:	46bd      	mov	sp, r7
 8003e3e:	bd80      	pop	{r7, pc}
 8003e40:	40010800 	.word	0x40010800
 8003e44:	40013800 	.word	0x40013800
 8003e48:	40004400 	.word	0x40004400

08003e4c <Default_Usart_Config>:
/**
  * @brief  Setup the default USART config stuff
  * @param  Init type pointer
  * @retval None
  */
void Default_Usart_Config(USART_InitTypeDef* init) {
 8003e4c:	b480      	push	{r7}
 8003e4e:	b083      	sub	sp, #12
 8003e50:	af00      	add	r7, sp, #0
 8003e52:	6078      	str	r0, [r7, #4]
    init->USART_WordLength = USART_WordLength_9b;
 8003e54:	687b      	ldr	r3, [r7, #4]
 8003e56:	f44f 5280 	mov.w	r2, #4096	; 0x1000
 8003e5a:	809a      	strh	r2, [r3, #4]
    init->USART_StopBits = USART_StopBits_1;
 8003e5c:	687b      	ldr	r3, [r7, #4]
 8003e5e:	f04f 0200 	mov.w	r2, #0
 8003e62:	80da      	strh	r2, [r3, #6]
    init->USART_Parity = USART_Parity_Even;
 8003e64:	687b      	ldr	r3, [r7, #4]
 8003e66:	f44f 6280 	mov.w	r2, #1024	; 0x400
 8003e6a:	811a      	strh	r2, [r3, #8]
    init->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 8003e6c:	687b      	ldr	r3, [r7, #4]
 8003e6e:	f04f 0200 	mov.w	r2, #0
 8003e72:	819a      	strh	r2, [r3, #12]
    init->USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
 8003e74:	687b      	ldr	r3, [r7, #4]
 8003e76:	f04f 020c 	mov.w	r2, #12
 8003e7a:	815a      	strh	r2, [r3, #10]
}
 8003e7c:	f107 070c 	add.w	r7, r7, #12
 8003e80:	46bd      	mov	sp, r7
 8003e82:	bc80      	pop	{r7}
 8003e84:	4770      	bx	lr
 8003e86:	bf00      	nop

08003e88 <Usart_Send_Str>:
/**
  * @brief  Writes a string to USART1
  * @param  String pointer - null terminated
  * @retval None
  */
void Usart_Send_Str(char* str) {
 8003e88:	b580      	push	{r7, lr}
 8003e8a:	b084      	sub	sp, #16
 8003e8c:	af00      	add	r7, sp, #0
 8003e8e:	6078      	str	r0, [r7, #4]
	unsigned short int i = 0;
 8003e90:	f04f 0300 	mov.w	r3, #0
 8003e94:	81fb      	strh	r3, [r7, #14]
	while(str[i] != 0x00)
 8003e96:	e00a      	b.n	8003eae <Usart_Send_Str+0x26>
		__usart_send_char(str[i++]);
 8003e98:	89fb      	ldrh	r3, [r7, #14]
 8003e9a:	687a      	ldr	r2, [r7, #4]
 8003e9c:	18d3      	adds	r3, r2, r3
 8003e9e:	781b      	ldrb	r3, [r3, #0]
 8003ea0:	89fa      	ldrh	r2, [r7, #14]
 8003ea2:	f102 0201 	add.w	r2, r2, #1
 8003ea6:	81fa      	strh	r2, [r7, #14]
 8003ea8:	4618      	mov	r0, r3
 8003eaa:	f000 f80b 	bl	8003ec4 <__usart_send_char>
  * @param  String pointer - null terminated
  * @retval None
  */
void Usart_Send_Str(char* str) {
	unsigned short int i = 0;
	while(str[i] != 0x00)
 8003eae:	89fb      	ldrh	r3, [r7, #14]
 8003eb0:	687a      	ldr	r2, [r7, #4]
 8003eb2:	18d3      	adds	r3, r2, r3
 8003eb4:	781b      	ldrb	r3, [r3, #0]
 8003eb6:	2b00      	cmp	r3, #0
 8003eb8:	d1ee      	bne.n	8003e98 <Usart_Send_Str+0x10>
		__usart_send_char(str[i++]);
}
 8003eba:	f107 0710 	add.w	r7, r7, #16
 8003ebe:	46bd      	mov	sp, r7
 8003ec0:	bd80      	pop	{r7, pc}
 8003ec2:	bf00      	nop

08003ec4 <__usart_send_char>:

#endif


//Private functions
void __usart_send_char(char data) {
 8003ec4:	b580      	push	{r7, lr}
 8003ec6:	b082      	sub	sp, #8
 8003ec8:	af00      	add	r7, sp, #0
 8003eca:	4603      	mov	r3, r0
 8003ecc:	71fb      	strb	r3, [r7, #7]
    USART_SendData(USART1_USART, data);
 8003ece:	79fb      	ldrb	r3, [r7, #7]
 8003ed0:	b29b      	uxth	r3, r3
 8003ed2:	4808      	ldr	r0, [pc, #32]	; (8003ef4 <__usart_send_char+0x30>)
 8003ed4:	4619      	mov	r1, r3
 8003ed6:	f001 f894 	bl	8005002 <USART_SendData>
    while(USART_GetFlagStatus(USART1_USART, USART_FLAG_TXE) == RESET) {}
 8003eda:	bf00      	nop
 8003edc:	4805      	ldr	r0, [pc, #20]	; (8003ef4 <__usart_send_char+0x30>)
 8003ede:	f04f 0180 	mov.w	r1, #128	; 0x80
 8003ee2:	f001 f8ef 	bl	80050c4 <USART_GetFlagStatus>
 8003ee6:	4603      	mov	r3, r0
 8003ee8:	2b00      	cmp	r3, #0
 8003eea:	d0f7      	beq.n	8003edc <__usart_send_char+0x18>
}
 8003eec:	f107 0708 	add.w	r7, r7, #8
 8003ef0:	46bd      	mov	sp, r7
 8003ef2:	bd80      	pop	{r7, pc}
 8003ef4:	40013800 	.word	0x40013800

08003ef8 <NVIC_PriorityGroupConfig>:
 8003ef8:	f040 60be 	orr.w	r0, r0, #99614720	; 0x5f00000
 8003efc:	4b02      	ldr	r3, [pc, #8]	; (8003f08 <NVIC_PriorityGroupConfig+0x10>)
 8003efe:	f440 2020 	orr.w	r0, r0, #655360	; 0xa0000
 8003f02:	60d8      	str	r0, [r3, #12]
 8003f04:	4770      	bx	lr
 8003f06:	bf00      	nop
 8003f08:	e000ed00 	.word	0xe000ed00

08003f0c <NVIC_Init>:
 8003f0c:	78c3      	ldrb	r3, [r0, #3]
 8003f0e:	b510      	push	{r4, lr}
 8003f10:	b1e3      	cbz	r3, 8003f4c <NVIC_Init+0x40>
 8003f12:	4b14      	ldr	r3, [pc, #80]	; (8003f64 <NVIC_Init+0x58>)
 8003f14:	7841      	ldrb	r1, [r0, #1]
 8003f16:	68db      	ldr	r3, [r3, #12]
 8003f18:	7884      	ldrb	r4, [r0, #2]
 8003f1a:	43db      	mvns	r3, r3
 8003f1c:	f403 63e0 	and.w	r3, r3, #1792	; 0x700
 8003f20:	0a1b      	lsrs	r3, r3, #8
 8003f22:	f1c3 0204 	rsb	r2, r3, #4
 8003f26:	4091      	lsls	r1, r2
 8003f28:	220f      	movs	r2, #15
 8003f2a:	40da      	lsrs	r2, r3
 8003f2c:	4022      	ands	r2, r4
 8003f2e:	430a      	orrs	r2, r1
 8003f30:	4b0d      	ldr	r3, [pc, #52]	; (8003f68 <NVIC_Init+0x5c>)
 8003f32:	7801      	ldrb	r1, [r0, #0]
 8003f34:	0112      	lsls	r2, r2, #4
 8003f36:	18cb      	adds	r3, r1, r3
 8003f38:	f883 2300 	strb.w	r2, [r3, #768]	; 0x300
 8003f3c:	7803      	ldrb	r3, [r0, #0]
 8003f3e:	2201      	movs	r2, #1
 8003f40:	0959      	lsrs	r1, r3, #5
 8003f42:	f003 031f 	and.w	r3, r3, #31
 8003f46:	fa12 f303 	lsls.w	r3, r2, r3
 8003f4a:	e007      	b.n	8003f5c <NVIC_Init+0x50>
 8003f4c:	7803      	ldrb	r3, [r0, #0]
 8003f4e:	2201      	movs	r2, #1
 8003f50:	0959      	lsrs	r1, r3, #5
 8003f52:	f003 031f 	and.w	r3, r3, #31
 8003f56:	fa12 f303 	lsls.w	r3, r2, r3
 8003f5a:	3120      	adds	r1, #32
 8003f5c:	4a02      	ldr	r2, [pc, #8]	; (8003f68 <NVIC_Init+0x5c>)
 8003f5e:	f842 3021 	str.w	r3, [r2, r1, lsl #2]
 8003f62:	bd10      	pop	{r4, pc}
 8003f64:	e000ed00 	.word	0xe000ed00
 8003f68:	e000e100 	.word	0xe000e100

08003f6c <NVIC_SetVectorTable>:
 8003f6c:	f021 4160 	bic.w	r1, r1, #3758096384	; 0xe0000000
 8003f70:	f021 017f 	bic.w	r1, r1, #127	; 0x7f
 8003f74:	4b01      	ldr	r3, [pc, #4]	; (8003f7c <NVIC_SetVectorTable+0x10>)
 8003f76:	4301      	orrs	r1, r0
 8003f78:	6099      	str	r1, [r3, #8]
 8003f7a:	4770      	bx	lr
 8003f7c:	e000ed00 	.word	0xe000ed00

08003f80 <NVIC_SystemLPConfig>:
 8003f80:	4b04      	ldr	r3, [pc, #16]	; (8003f94 <NVIC_SystemLPConfig+0x14>)
 8003f82:	b111      	cbz	r1, 8003f8a <NVIC_SystemLPConfig+0xa>
 8003f84:	691a      	ldr	r2, [r3, #16]
 8003f86:	4310      	orrs	r0, r2
 8003f88:	e002      	b.n	8003f90 <NVIC_SystemLPConfig+0x10>
 8003f8a:	691a      	ldr	r2, [r3, #16]
 8003f8c:	ea22 0000 	bic.w	r0, r2, r0
 8003f90:	6118      	str	r0, [r3, #16]
 8003f92:	4770      	bx	lr
 8003f94:	e000ed00 	.word	0xe000ed00

08003f98 <SysTick_CLKSourceConfig>:
 8003f98:	4b04      	ldr	r3, [pc, #16]	; (8003fac <SysTick_CLKSourceConfig+0x14>)
 8003f9a:	2804      	cmp	r0, #4
 8003f9c:	681a      	ldr	r2, [r3, #0]
 8003f9e:	bf0c      	ite	eq
 8003fa0:	f042 0204 	orreq.w	r2, r2, #4
 8003fa4:	f022 0204 	bicne.w	r2, r2, #4
 8003fa8:	601a      	str	r2, [r3, #0]
 8003faa:	4770      	bx	lr
 8003fac:	e000e010 	.word	0xe000e010

08003fb0 <ADC_DeInit>:
 8003fb0:	4b13      	ldr	r3, [pc, #76]	; (8004000 <ADC_DeInit+0x50>)
 8003fb2:	b510      	push	{r4, lr}
 8003fb4:	4298      	cmp	r0, r3
 8003fb6:	d107      	bne.n	8003fc8 <ADC_DeInit+0x18>
 8003fb8:	f44f 7000 	mov.w	r0, #512	; 0x200
 8003fbc:	2101      	movs	r1, #1
 8003fbe:	f000 fd21 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 8003fc2:	f44f 7000 	mov.w	r0, #512	; 0x200
 8003fc6:	e014      	b.n	8003ff2 <ADC_DeInit+0x42>
 8003fc8:	4b0e      	ldr	r3, [pc, #56]	; (8004004 <ADC_DeInit+0x54>)
 8003fca:	4298      	cmp	r0, r3
 8003fcc:	d107      	bne.n	8003fde <ADC_DeInit+0x2e>
 8003fce:	f44f 6080 	mov.w	r0, #1024	; 0x400
 8003fd2:	2101      	movs	r1, #1
 8003fd4:	f000 fd16 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 8003fd8:	f44f 6080 	mov.w	r0, #1024	; 0x400
 8003fdc:	e009      	b.n	8003ff2 <ADC_DeInit+0x42>
 8003fde:	4b0a      	ldr	r3, [pc, #40]	; (8004008 <ADC_DeInit+0x58>)
 8003fe0:	4298      	cmp	r0, r3
 8003fe2:	d10b      	bne.n	8003ffc <ADC_DeInit+0x4c>
 8003fe4:	f44f 4000 	mov.w	r0, #32768	; 0x8000
 8003fe8:	2101      	movs	r1, #1
 8003fea:	f000 fd0b 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 8003fee:	f44f 4000 	mov.w	r0, #32768	; 0x8000
 8003ff2:	2100      	movs	r1, #0
 8003ff4:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 8003ff8:	f000 bd04 	b.w	8004a04 <RCC_APB2PeriphResetCmd>
 8003ffc:	bd10      	pop	{r4, pc}
 8003ffe:	bf00      	nop
 8004000:	40012400 	.word	0x40012400
 8004004:	40012800 	.word	0x40012800
 8004008:	40013c00 	.word	0x40013c00

0800400c <ADC_Init>:
 800400c:	6843      	ldr	r3, [r0, #4]
 800400e:	680a      	ldr	r2, [r1, #0]
 8004010:	f423 2370 	bic.w	r3, r3, #983040	; 0xf0000
 8004014:	f423 7380 	bic.w	r3, r3, #256	; 0x100
 8004018:	4313      	orrs	r3, r2
 800401a:	790a      	ldrb	r2, [r1, #4]
 800401c:	b510      	push	{r4, lr}
 800401e:	ea43 2302 	orr.w	r3, r3, r2, lsl #8
 8004022:	6043      	str	r3, [r0, #4]
 8004024:	68cb      	ldr	r3, [r1, #12]
 8004026:	688a      	ldr	r2, [r1, #8]
 8004028:	6884      	ldr	r4, [r0, #8]
 800402a:	431a      	orrs	r2, r3
 800402c:	4b09      	ldr	r3, [pc, #36]	; (8004054 <ADC_Init+0x48>)
 800402e:	ea04 0303 	and.w	r3, r4, r3
 8004032:	ea42 0303 	orr.w	r3, r2, r3
 8004036:	794a      	ldrb	r2, [r1, #5]
 8004038:	ea43 0342 	orr.w	r3, r3, r2, lsl #1
 800403c:	6083      	str	r3, [r0, #8]
 800403e:	7c0b      	ldrb	r3, [r1, #16]
 8004040:	6ac2      	ldr	r2, [r0, #44]	; 0x2c
 8004042:	3b01      	subs	r3, #1
 8004044:	f422 0270 	bic.w	r2, r2, #15728640	; 0xf00000
 8004048:	b2db      	uxtb	r3, r3
 800404a:	ea42 5303 	orr.w	r3, r2, r3, lsl #20
 800404e:	62c3      	str	r3, [r0, #44]	; 0x2c
 8004050:	bd10      	pop	{r4, pc}
 8004052:	bf00      	nop
 8004054:	fff1f7fd 	.word	0xfff1f7fd

08004058 <ADC_StructInit>:
 8004058:	2300      	movs	r3, #0
 800405a:	6003      	str	r3, [r0, #0]
 800405c:	7103      	strb	r3, [r0, #4]
 800405e:	7143      	strb	r3, [r0, #5]
 8004060:	6083      	str	r3, [r0, #8]
 8004062:	60c3      	str	r3, [r0, #12]
 8004064:	3301      	adds	r3, #1
 8004066:	7403      	strb	r3, [r0, #16]
 8004068:	4770      	bx	lr

0800406a <ADC_Cmd>:
 800406a:	b119      	cbz	r1, 8004074 <ADC_Cmd+0xa>
 800406c:	6883      	ldr	r3, [r0, #8]
 800406e:	f043 0301 	orr.w	r3, r3, #1
 8004072:	e002      	b.n	800407a <ADC_Cmd+0x10>
 8004074:	6883      	ldr	r3, [r0, #8]
 8004076:	f023 0301 	bic.w	r3, r3, #1
 800407a:	6083      	str	r3, [r0, #8]
 800407c:	4770      	bx	lr

0800407e <ADC_DMACmd>:
 800407e:	b119      	cbz	r1, 8004088 <ADC_DMACmd+0xa>
 8004080:	6883      	ldr	r3, [r0, #8]
 8004082:	f443 7380 	orr.w	r3, r3, #256	; 0x100
 8004086:	e002      	b.n	800408e <ADC_DMACmd+0x10>
 8004088:	6883      	ldr	r3, [r0, #8]
 800408a:	f423 7380 	bic.w	r3, r3, #256	; 0x100
 800408e:	6083      	str	r3, [r0, #8]
 8004090:	4770      	bx	lr

08004092 <ADC_ITConfig>:
 8004092:	b2c9      	uxtb	r1, r1
 8004094:	b112      	cbz	r2, 800409c <ADC_ITConfig+0xa>
 8004096:	6843      	ldr	r3, [r0, #4]
 8004098:	4319      	orrs	r1, r3
 800409a:	e002      	b.n	80040a2 <ADC_ITConfig+0x10>
 800409c:	6843      	ldr	r3, [r0, #4]
 800409e:	ea23 0101 	bic.w	r1, r3, r1
 80040a2:	6041      	str	r1, [r0, #4]
 80040a4:	4770      	bx	lr

080040a6 <ADC_ResetCalibration>:
 80040a6:	6883      	ldr	r3, [r0, #8]
 80040a8:	f043 0308 	orr.w	r3, r3, #8
 80040ac:	6083      	str	r3, [r0, #8]
 80040ae:	4770      	bx	lr

080040b0 <ADC_GetResetCalibrationStatus>:
 80040b0:	6880      	ldr	r0, [r0, #8]
 80040b2:	f3c0 00c0 	ubfx	r0, r0, #3, #1
 80040b6:	4770      	bx	lr

080040b8 <ADC_StartCalibration>:
 80040b8:	6883      	ldr	r3, [r0, #8]
 80040ba:	f043 0304 	orr.w	r3, r3, #4
 80040be:	6083      	str	r3, [r0, #8]
 80040c0:	4770      	bx	lr

080040c2 <ADC_GetCalibrationStatus>:
 80040c2:	6880      	ldr	r0, [r0, #8]
 80040c4:	f3c0 0080 	ubfx	r0, r0, #2, #1
 80040c8:	4770      	bx	lr

080040ca <ADC_SoftwareStartConvCmd>:
 80040ca:	b119      	cbz	r1, 80040d4 <ADC_SoftwareStartConvCmd+0xa>
 80040cc:	6883      	ldr	r3, [r0, #8]
 80040ce:	f443 03a0 	orr.w	r3, r3, #5242880	; 0x500000
 80040d2:	e002      	b.n	80040da <ADC_SoftwareStartConvCmd+0x10>
 80040d4:	6883      	ldr	r3, [r0, #8]
 80040d6:	f423 03a0 	bic.w	r3, r3, #5242880	; 0x500000
 80040da:	6083      	str	r3, [r0, #8]
 80040dc:	4770      	bx	lr

080040de <ADC_GetSoftwareStartConvStatus>:
 80040de:	6880      	ldr	r0, [r0, #8]
 80040e0:	f3c0 5080 	ubfx	r0, r0, #22, #1
 80040e4:	4770      	bx	lr

080040e6 <ADC_DiscModeChannelCountConfig>:
 80040e6:	6843      	ldr	r3, [r0, #4]
 80040e8:	3901      	subs	r1, #1
 80040ea:	f423 4360 	bic.w	r3, r3, #57344	; 0xe000
 80040ee:	ea43 3341 	orr.w	r3, r3, r1, lsl #13
 80040f2:	6043      	str	r3, [r0, #4]
 80040f4:	4770      	bx	lr

080040f6 <ADC_DiscModeCmd>:
 80040f6:	b119      	cbz	r1, 8004100 <ADC_DiscModeCmd+0xa>
 80040f8:	6843      	ldr	r3, [r0, #4]
 80040fa:	f443 6300 	orr.w	r3, r3, #2048	; 0x800
 80040fe:	e002      	b.n	8004106 <ADC_DiscModeCmd+0x10>
 8004100:	6843      	ldr	r3, [r0, #4]
 8004102:	f423 6300 	bic.w	r3, r3, #2048	; 0x800
 8004106:	6043      	str	r3, [r0, #4]
 8004108:	4770      	bx	lr

0800410a <ADC_RegularChannelConfig>:
 800410a:	2909      	cmp	r1, #9
 800410c:	b570      	push	{r4, r5, r6, lr}
 800410e:	ea4f 0441 	mov.w	r4, r1, lsl #1
 8004112:	d90b      	bls.n	800412c <ADC_RegularChannelConfig+0x22>
 8004114:	1864      	adds	r4, r4, r1
 8004116:	3c1e      	subs	r4, #30
 8004118:	2507      	movs	r5, #7
 800411a:	40a5      	lsls	r5, r4
 800411c:	40a3      	lsls	r3, r4
 800411e:	68c6      	ldr	r6, [r0, #12]
 8004120:	ea26 0505 	bic.w	r5, r6, r5
 8004124:	ea45 0403 	orr.w	r4, r5, r3
 8004128:	60c4      	str	r4, [r0, #12]
 800412a:	e00a      	b.n	8004142 <ADC_RegularChannelConfig+0x38>
 800412c:	1864      	adds	r4, r4, r1
 800412e:	2507      	movs	r5, #7
 8004130:	40a5      	lsls	r5, r4
 8004132:	fa13 f404 	lsls.w	r4, r3, r4
 8004136:	6906      	ldr	r6, [r0, #16]
 8004138:	ea26 0505 	bic.w	r5, r6, r5
 800413c:	ea45 0304 	orr.w	r3, r5, r4
 8004140:	6103      	str	r3, [r0, #16]
 8004142:	2a06      	cmp	r2, #6
 8004144:	d80c      	bhi.n	8004160 <ADC_RegularChannelConfig+0x56>
 8004146:	3a01      	subs	r2, #1
 8004148:	eb02 0282 	add.w	r2, r2, r2, lsl #2
 800414c:	231f      	movs	r3, #31
 800414e:	4093      	lsls	r3, r2
 8004150:	4091      	lsls	r1, r2
 8004152:	6b44      	ldr	r4, [r0, #52]	; 0x34
 8004154:	ea24 0303 	bic.w	r3, r4, r3
 8004158:	ea43 0201 	orr.w	r2, r3, r1
 800415c:	6342      	str	r2, [r0, #52]	; 0x34
 800415e:	bd70      	pop	{r4, r5, r6, pc}
 8004160:	2a0c      	cmp	r2, #12
 8004162:	ea4f 0382 	mov.w	r3, r2, lsl #2
 8004166:	d80b      	bhi.n	8004180 <ADC_RegularChannelConfig+0x76>
 8004168:	189a      	adds	r2, r3, r2
 800416a:	3a23      	subs	r2, #35	; 0x23
 800416c:	231f      	movs	r3, #31
 800416e:	4093      	lsls	r3, r2
 8004170:	4091      	lsls	r1, r2
 8004172:	6b04      	ldr	r4, [r0, #48]	; 0x30
 8004174:	ea24 0303 	bic.w	r3, r4, r3
 8004178:	ea43 0201 	orr.w	r2, r3, r1
 800417c:	6302      	str	r2, [r0, #48]	; 0x30
 800417e:	bd70      	pop	{r4, r5, r6, pc}
 8004180:	189a      	adds	r2, r3, r2
 8004182:	3a41      	subs	r2, #65	; 0x41
 8004184:	231f      	movs	r3, #31
 8004186:	4093      	lsls	r3, r2
 8004188:	4091      	lsls	r1, r2
 800418a:	6ac4      	ldr	r4, [r0, #44]	; 0x2c
 800418c:	ea24 0303 	bic.w	r3, r4, r3
 8004190:	ea43 0201 	orr.w	r2, r3, r1
 8004194:	62c2      	str	r2, [r0, #44]	; 0x2c
 8004196:	bd70      	pop	{r4, r5, r6, pc}

08004198 <ADC_ExternalTrigConvCmd>:
 8004198:	b119      	cbz	r1, 80041a2 <ADC_ExternalTrigConvCmd+0xa>
 800419a:	6883      	ldr	r3, [r0, #8]
 800419c:	f443 1380 	orr.w	r3, r3, #1048576	; 0x100000
 80041a0:	e002      	b.n	80041a8 <ADC_ExternalTrigConvCmd+0x10>
 80041a2:	6883      	ldr	r3, [r0, #8]
 80041a4:	f423 1380 	bic.w	r3, r3, #1048576	; 0x100000
 80041a8:	6083      	str	r3, [r0, #8]
 80041aa:	4770      	bx	lr

080041ac <ADC_GetConversionValue>:
 80041ac:	6cc0      	ldr	r0, [r0, #76]	; 0x4c
 80041ae:	b280      	uxth	r0, r0
 80041b0:	4770      	bx	lr

080041b2 <ADC_GetDualModeConversionValue>:
 80041b2:	4b01      	ldr	r3, [pc, #4]	; (80041b8 <ADC_GetDualModeConversionValue+0x6>)
 80041b4:	6818      	ldr	r0, [r3, #0]
 80041b6:	4770      	bx	lr
 80041b8:	4001244c 	.word	0x4001244c

080041bc <ADC_AutoInjectedConvCmd>:
 80041bc:	b119      	cbz	r1, 80041c6 <ADC_AutoInjectedConvCmd+0xa>
 80041be:	6843      	ldr	r3, [r0, #4]
 80041c0:	f443 6380 	orr.w	r3, r3, #1024	; 0x400
 80041c4:	e002      	b.n	80041cc <ADC_AutoInjectedConvCmd+0x10>
 80041c6:	6843      	ldr	r3, [r0, #4]
 80041c8:	f423 6380 	bic.w	r3, r3, #1024	; 0x400
 80041cc:	6043      	str	r3, [r0, #4]
 80041ce:	4770      	bx	lr

080041d0 <ADC_InjectedDiscModeCmd>:
 80041d0:	b119      	cbz	r1, 80041da <ADC_InjectedDiscModeCmd+0xa>
 80041d2:	6843      	ldr	r3, [r0, #4]
 80041d4:	f443 5380 	orr.w	r3, r3, #4096	; 0x1000
 80041d8:	e002      	b.n	80041e0 <ADC_InjectedDiscModeCmd+0x10>
 80041da:	6843      	ldr	r3, [r0, #4]
 80041dc:	f423 5380 	bic.w	r3, r3, #4096	; 0x1000
 80041e0:	6043      	str	r3, [r0, #4]
 80041e2:	4770      	bx	lr

080041e4 <ADC_ExternalTrigInjectedConvConfig>:
 80041e4:	6883      	ldr	r3, [r0, #8]
 80041e6:	f423 43e0 	bic.w	r3, r3, #28672	; 0x7000
 80041ea:	ea41 0303 	orr.w	r3, r1, r3
 80041ee:	6083      	str	r3, [r0, #8]
 80041f0:	4770      	bx	lr

080041f2 <ADC_ExternalTrigInjectedConvCmd>:
 80041f2:	b119      	cbz	r1, 80041fc <ADC_ExternalTrigInjectedConvCmd+0xa>
 80041f4:	6883      	ldr	r3, [r0, #8]
 80041f6:	f443 4300 	orr.w	r3, r3, #32768	; 0x8000
 80041fa:	e002      	b.n	8004202 <ADC_ExternalTrigInjectedConvCmd+0x10>
 80041fc:	6883      	ldr	r3, [r0, #8]
 80041fe:	f423 4300 	bic.w	r3, r3, #32768	; 0x8000
 8004202:	6083      	str	r3, [r0, #8]
 8004204:	4770      	bx	lr

08004206 <ADC_SoftwareStartInjectedConvCmd>:
 8004206:	b119      	cbz	r1, 8004210 <ADC_SoftwareStartInjectedConvCmd+0xa>
 8004208:	6883      	ldr	r3, [r0, #8]
 800420a:	f443 1302 	orr.w	r3, r3, #2129920	; 0x208000
 800420e:	e002      	b.n	8004216 <ADC_SoftwareStartInjectedConvCmd+0x10>
 8004210:	6883      	ldr	r3, [r0, #8]
 8004212:	f423 1302 	bic.w	r3, r3, #2129920	; 0x208000
 8004216:	6083      	str	r3, [r0, #8]
 8004218:	4770      	bx	lr

0800421a <ADC_GetSoftwareStartInjectedConvCmdStatus>:
 800421a:	6880      	ldr	r0, [r0, #8]
 800421c:	f3c0 5040 	ubfx	r0, r0, #21, #1
 8004220:	4770      	bx	lr

08004222 <ADC_InjectedChannelConfig>:
 8004222:	2909      	cmp	r1, #9
 8004224:	b570      	push	{r4, r5, r6, lr}
 8004226:	ea4f 0441 	mov.w	r4, r1, lsl #1
 800422a:	d90b      	bls.n	8004244 <ADC_InjectedChannelConfig+0x22>
 800422c:	1864      	adds	r4, r4, r1
 800422e:	3c1e      	subs	r4, #30
 8004230:	2507      	movs	r5, #7
 8004232:	40a5      	lsls	r5, r4
 8004234:	40a3      	lsls	r3, r4
 8004236:	68c6      	ldr	r6, [r0, #12]
 8004238:	ea26 0505 	bic.w	r5, r6, r5
 800423c:	ea45 0403 	orr.w	r4, r5, r3
 8004240:	60c4      	str	r4, [r0, #12]
 8004242:	e00a      	b.n	800425a <ADC_InjectedChannelConfig+0x38>
 8004244:	1864      	adds	r4, r4, r1
 8004246:	2507      	movs	r5, #7
 8004248:	40a5      	lsls	r5, r4
 800424a:	fa13 f404 	lsls.w	r4, r3, r4
 800424e:	6906      	ldr	r6, [r0, #16]
 8004250:	ea26 0505 	bic.w	r5, r6, r5
 8004254:	ea45 0304 	orr.w	r3, r5, r4
 8004258:	6103      	str	r3, [r0, #16]
 800425a:	6b83      	ldr	r3, [r0, #56]	; 0x38
 800425c:	f403 1440 	and.w	r4, r3, #3145728	; 0x300000
 8004260:	ea6f 5414 	mvn.w	r4, r4, lsr #20
 8004264:	1912      	adds	r2, r2, r4
 8004266:	3203      	adds	r2, #3
 8004268:	b2d2      	uxtb	r2, r2
 800426a:	eb02 0282 	add.w	r2, r2, r2, lsl #2
 800426e:	241f      	movs	r4, #31
 8004270:	4094      	lsls	r4, r2
 8004272:	4091      	lsls	r1, r2
 8004274:	ea23 0304 	bic.w	r3, r3, r4
 8004278:	430b      	orrs	r3, r1
 800427a:	6383      	str	r3, [r0, #56]	; 0x38
 800427c:	bd70      	pop	{r4, r5, r6, pc}

0800427e <ADC_InjectedSequencerLengthConfig>:
 800427e:	6b83      	ldr	r3, [r0, #56]	; 0x38
 8004280:	3901      	subs	r1, #1
 8004282:	f423 1340 	bic.w	r3, r3, #3145728	; 0x300000
 8004286:	ea43 5301 	orr.w	r3, r3, r1, lsl #20
 800428a:	6383      	str	r3, [r0, #56]	; 0x38
 800428c:	4770      	bx	lr

0800428e <ADC_SetInjectedOffset>:
 800428e:	b082      	sub	sp, #8
 8004290:	2300      	movs	r3, #0
 8004292:	9301      	str	r3, [sp, #4]
 8004294:	9001      	str	r0, [sp, #4]
 8004296:	9b01      	ldr	r3, [sp, #4]
 8004298:	18cb      	adds	r3, r1, r3
 800429a:	9301      	str	r3, [sp, #4]
 800429c:	9b01      	ldr	r3, [sp, #4]
 800429e:	601a      	str	r2, [r3, #0]
 80042a0:	b002      	add	sp, #8
 80042a2:	4770      	bx	lr

080042a4 <ADC_GetInjectedConversionValue>:
 80042a4:	b082      	sub	sp, #8
 80042a6:	2300      	movs	r3, #0
 80042a8:	9301      	str	r3, [sp, #4]
 80042aa:	9001      	str	r0, [sp, #4]
 80042ac:	9b01      	ldr	r3, [sp, #4]
 80042ae:	3328      	adds	r3, #40	; 0x28
 80042b0:	185b      	adds	r3, r3, r1
 80042b2:	9301      	str	r3, [sp, #4]
 80042b4:	9b01      	ldr	r3, [sp, #4]
 80042b6:	6818      	ldr	r0, [r3, #0]
 80042b8:	b280      	uxth	r0, r0
 80042ba:	b002      	add	sp, #8
 80042bc:	4770      	bx	lr

080042be <ADC_AnalogWatchdogCmd>:
 80042be:	6843      	ldr	r3, [r0, #4]
 80042c0:	f423 0340 	bic.w	r3, r3, #12582912	; 0xc00000
 80042c4:	f423 7300 	bic.w	r3, r3, #512	; 0x200
 80042c8:	ea41 0303 	orr.w	r3, r1, r3
 80042cc:	6043      	str	r3, [r0, #4]
 80042ce:	4770      	bx	lr

080042d0 <ADC_AnalogWatchdogThresholdsConfig>:
 80042d0:	6241      	str	r1, [r0, #36]	; 0x24
 80042d2:	6282      	str	r2, [r0, #40]	; 0x28
 80042d4:	4770      	bx	lr

080042d6 <ADC_AnalogWatchdogSingleChannelConfig>:
 80042d6:	6843      	ldr	r3, [r0, #4]
 80042d8:	f023 031f 	bic.w	r3, r3, #31
 80042dc:	ea41 0303 	orr.w	r3, r1, r3
 80042e0:	6043      	str	r3, [r0, #4]
 80042e2:	4770      	bx	lr

080042e4 <ADC_TempSensorVrefintCmd>:
 80042e4:	4b05      	ldr	r3, [pc, #20]	; (80042fc <ADC_TempSensorVrefintCmd+0x18>)
 80042e6:	b118      	cbz	r0, 80042f0 <ADC_TempSensorVrefintCmd+0xc>
 80042e8:	689a      	ldr	r2, [r3, #8]
 80042ea:	f442 0200 	orr.w	r2, r2, #8388608	; 0x800000
 80042ee:	e002      	b.n	80042f6 <ADC_TempSensorVrefintCmd+0x12>
 80042f0:	689a      	ldr	r2, [r3, #8]
 80042f2:	f422 0200 	bic.w	r2, r2, #8388608	; 0x800000
 80042f6:	609a      	str	r2, [r3, #8]
 80042f8:	4770      	bx	lr
 80042fa:	bf00      	nop
 80042fc:	40012400 	.word	0x40012400

08004300 <ADC_GetFlagStatus>:
 8004300:	6803      	ldr	r3, [r0, #0]
 8004302:	4219      	tst	r1, r3
 8004304:	bf0c      	ite	eq
 8004306:	2000      	moveq	r0, #0
 8004308:	2001      	movne	r0, #1
 800430a:	4770      	bx	lr

0800430c <ADC_ClearFlag>:
 800430c:	43c9      	mvns	r1, r1
 800430e:	6001      	str	r1, [r0, #0]
 8004310:	4770      	bx	lr

08004312 <ADC_GetITStatus>:
 8004312:	6843      	ldr	r3, [r0, #4]
 8004314:	6800      	ldr	r0, [r0, #0]
 8004316:	ea10 2011 	ands.w	r0, r0, r1, lsr #8
 800431a:	d004      	beq.n	8004326 <ADC_GetITStatus+0x14>
 800431c:	b2c9      	uxtb	r1, r1
 800431e:	4219      	tst	r1, r3
 8004320:	bf0c      	ite	eq
 8004322:	2000      	moveq	r0, #0
 8004324:	2001      	movne	r0, #1
 8004326:	4770      	bx	lr

08004328 <ADC_ClearITPendingBit>:
 8004328:	ea6f 2111 	mvn.w	r1, r1, lsr #8
 800432c:	6001      	str	r1, [r0, #0]
 800432e:	4770      	bx	lr

08004330 <EXTI_DeInit>:
 8004330:	4b04      	ldr	r3, [pc, #16]	; (8004344 <EXTI_DeInit+0x14>)
 8004332:	2200      	movs	r2, #0
 8004334:	601a      	str	r2, [r3, #0]
 8004336:	605a      	str	r2, [r3, #4]
 8004338:	609a      	str	r2, [r3, #8]
 800433a:	60da      	str	r2, [r3, #12]
 800433c:	4a02      	ldr	r2, [pc, #8]	; (8004348 <EXTI_DeInit+0x18>)
 800433e:	615a      	str	r2, [r3, #20]
 8004340:	4770      	bx	lr
 8004342:	bf00      	nop
 8004344:	40010400 	.word	0x40010400
 8004348:	000fffff 	.word	0x000fffff

0800434c <EXTI_Init>:
 800434c:	7983      	ldrb	r3, [r0, #6]
 800434e:	b510      	push	{r4, lr}
 8004350:	6802      	ldr	r2, [r0, #0]
 8004352:	2b00      	cmp	r3, #0
 8004354:	d030      	beq.n	80043b8 <EXTI_Init+0x6c>
 8004356:	4b1d      	ldr	r3, [pc, #116]	; (80043cc <EXTI_Init+0x80>)
 8004358:	6819      	ldr	r1, [r3, #0]
 800435a:	ea21 0202 	bic.w	r2, r1, r2
 800435e:	601a      	str	r2, [r3, #0]
 8004360:	685a      	ldr	r2, [r3, #4]
 8004362:	6801      	ldr	r1, [r0, #0]
 8004364:	ea22 0201 	bic.w	r2, r2, r1
 8004368:	605a      	str	r2, [r3, #4]
 800436a:	7902      	ldrb	r2, [r0, #4]
 800436c:	6801      	ldr	r1, [r0, #0]
 800436e:	18d2      	adds	r2, r2, r3
 8004370:	6814      	ldr	r4, [r2, #0]
 8004372:	ea44 0101 	orr.w	r1, r4, r1
 8004376:	6011      	str	r1, [r2, #0]
 8004378:	689a      	ldr	r2, [r3, #8]
 800437a:	6801      	ldr	r1, [r0, #0]
 800437c:	ea22 0201 	bic.w	r2, r2, r1
 8004380:	609a      	str	r2, [r3, #8]
 8004382:	6801      	ldr	r1, [r0, #0]
 8004384:	68da      	ldr	r2, [r3, #12]
 8004386:	ea22 0201 	bic.w	r2, r2, r1
 800438a:	60da      	str	r2, [r3, #12]
 800438c:	7941      	ldrb	r1, [r0, #5]
 800438e:	6802      	ldr	r2, [r0, #0]
 8004390:	2910      	cmp	r1, #16
 8004392:	d109      	bne.n	80043a8 <EXTI_Init+0x5c>
 8004394:	6899      	ldr	r1, [r3, #8]
 8004396:	ea41 0202 	orr.w	r2, r1, r2
 800439a:	609a      	str	r2, [r3, #8]
 800439c:	68d9      	ldr	r1, [r3, #12]
 800439e:	6802      	ldr	r2, [r0, #0]
 80043a0:	ea41 0202 	orr.w	r2, r1, r2
 80043a4:	60da      	str	r2, [r3, #12]
 80043a6:	bd10      	pop	{r4, pc}
 80043a8:	f101 4380 	add.w	r3, r1, #1073741824	; 0x40000000
 80043ac:	f503 3382 	add.w	r3, r3, #66560	; 0x10400
 80043b0:	6819      	ldr	r1, [r3, #0]
 80043b2:	ea41 0202 	orr.w	r2, r1, r2
 80043b6:	e007      	b.n	80043c8 <EXTI_Init+0x7c>
 80043b8:	7903      	ldrb	r3, [r0, #4]
 80043ba:	f103 4380 	add.w	r3, r3, #1073741824	; 0x40000000
 80043be:	f503 3382 	add.w	r3, r3, #66560	; 0x10400
 80043c2:	6819      	ldr	r1, [r3, #0]
 80043c4:	ea21 0202 	bic.w	r2, r1, r2
 80043c8:	601a      	str	r2, [r3, #0]
 80043ca:	bd10      	pop	{r4, pc}
 80043cc:	40010400 	.word	0x40010400

080043d0 <EXTI_StructInit>:
 80043d0:	2300      	movs	r3, #0
 80043d2:	220c      	movs	r2, #12
 80043d4:	6003      	str	r3, [r0, #0]
 80043d6:	7103      	strb	r3, [r0, #4]
 80043d8:	7142      	strb	r2, [r0, #5]
 80043da:	7183      	strb	r3, [r0, #6]
 80043dc:	4770      	bx	lr

080043de <EXTI_GenerateSWInterrupt>:
 80043de:	4b03      	ldr	r3, [pc, #12]	; (80043ec <EXTI_GenerateSWInterrupt+0xe>)
 80043e0:	691a      	ldr	r2, [r3, #16]
 80043e2:	ea40 0202 	orr.w	r2, r0, r2
 80043e6:	611a      	str	r2, [r3, #16]
 80043e8:	4770      	bx	lr
 80043ea:	bf00      	nop
 80043ec:	40010400 	.word	0x40010400

080043f0 <EXTI_GetFlagStatus>:
 80043f0:	4b03      	ldr	r3, [pc, #12]	; (8004400 <EXTI_GetFlagStatus+0x10>)
 80043f2:	695b      	ldr	r3, [r3, #20]
 80043f4:	4218      	tst	r0, r3
 80043f6:	bf0c      	ite	eq
 80043f8:	2000      	moveq	r0, #0
 80043fa:	2001      	movne	r0, #1
 80043fc:	4770      	bx	lr
 80043fe:	bf00      	nop
 8004400:	40010400 	.word	0x40010400

08004404 <EXTI_ClearFlag>:
 8004404:	4b01      	ldr	r3, [pc, #4]	; (800440c <EXTI_ClearFlag+0x8>)
 8004406:	6158      	str	r0, [r3, #20]
 8004408:	4770      	bx	lr
 800440a:	bf00      	nop
 800440c:	40010400 	.word	0x40010400

08004410 <EXTI_GetITStatus>:
 8004410:	4b05      	ldr	r3, [pc, #20]	; (8004428 <EXTI_GetITStatus+0x18>)
 8004412:	681a      	ldr	r2, [r3, #0]
 8004414:	695b      	ldr	r3, [r3, #20]
 8004416:	4003      	ands	r3, r0
 8004418:	d004      	beq.n	8004424 <EXTI_GetITStatus+0x14>
 800441a:	4210      	tst	r0, r2
 800441c:	bf0c      	ite	eq
 800441e:	2000      	moveq	r0, #0
 8004420:	2001      	movne	r0, #1
 8004422:	4770      	bx	lr
 8004424:	4618      	mov	r0, r3
 8004426:	4770      	bx	lr
 8004428:	40010400 	.word	0x40010400

0800442c <EXTI_ClearITPendingBit>:
 800442c:	4b01      	ldr	r3, [pc, #4]	; (8004434 <EXTI_ClearITPendingBit+0x8>)
 800442e:	6158      	str	r0, [r3, #20]
 8004430:	4770      	bx	lr
 8004432:	bf00      	nop
 8004434:	40010400 	.word	0x40010400

08004438 <GPIO_DeInit>:
 8004438:	4b23      	ldr	r3, [pc, #140]	; (80044c8 <GPIO_DeInit+0x90>)
 800443a:	b510      	push	{r4, lr}
 800443c:	4298      	cmp	r0, r3
 800443e:	d105      	bne.n	800444c <GPIO_DeInit+0x14>
 8004440:	2004      	movs	r0, #4
 8004442:	2101      	movs	r1, #1
 8004444:	f000 fade 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 8004448:	2004      	movs	r0, #4
 800444a:	e036      	b.n	80044ba <GPIO_DeInit+0x82>
 800444c:	4b1f      	ldr	r3, [pc, #124]	; (80044cc <GPIO_DeInit+0x94>)
 800444e:	4298      	cmp	r0, r3
 8004450:	d105      	bne.n	800445e <GPIO_DeInit+0x26>
 8004452:	2008      	movs	r0, #8
 8004454:	2101      	movs	r1, #1
 8004456:	f000 fad5 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 800445a:	2008      	movs	r0, #8
 800445c:	e02d      	b.n	80044ba <GPIO_DeInit+0x82>
 800445e:	4b1c      	ldr	r3, [pc, #112]	; (80044d0 <GPIO_DeInit+0x98>)
 8004460:	4298      	cmp	r0, r3
 8004462:	d105      	bne.n	8004470 <GPIO_DeInit+0x38>
 8004464:	2010      	movs	r0, #16
 8004466:	2101      	movs	r1, #1
 8004468:	f000 facc 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 800446c:	2010      	movs	r0, #16
 800446e:	e024      	b.n	80044ba <GPIO_DeInit+0x82>
 8004470:	4b18      	ldr	r3, [pc, #96]	; (80044d4 <GPIO_DeInit+0x9c>)
 8004472:	4298      	cmp	r0, r3
 8004474:	d105      	bne.n	8004482 <GPIO_DeInit+0x4a>
 8004476:	2020      	movs	r0, #32
 8004478:	2101      	movs	r1, #1
 800447a:	f000 fac3 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 800447e:	2020      	movs	r0, #32
 8004480:	e01b      	b.n	80044ba <GPIO_DeInit+0x82>
 8004482:	4b15      	ldr	r3, [pc, #84]	; (80044d8 <GPIO_DeInit+0xa0>)
 8004484:	4298      	cmp	r0, r3
 8004486:	d105      	bne.n	8004494 <GPIO_DeInit+0x5c>
 8004488:	2040      	movs	r0, #64	; 0x40
 800448a:	2101      	movs	r1, #1
 800448c:	f000 faba 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 8004490:	2040      	movs	r0, #64	; 0x40
 8004492:	e012      	b.n	80044ba <GPIO_DeInit+0x82>
 8004494:	4b11      	ldr	r3, [pc, #68]	; (80044dc <GPIO_DeInit+0xa4>)
 8004496:	4298      	cmp	r0, r3
 8004498:	d105      	bne.n	80044a6 <GPIO_DeInit+0x6e>
 800449a:	2080      	movs	r0, #128	; 0x80
 800449c:	2101      	movs	r1, #1
 800449e:	f000 fab1 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 80044a2:	2080      	movs	r0, #128	; 0x80
 80044a4:	e009      	b.n	80044ba <GPIO_DeInit+0x82>
 80044a6:	4b0e      	ldr	r3, [pc, #56]	; (80044e0 <GPIO_DeInit+0xa8>)
 80044a8:	4298      	cmp	r0, r3
 80044aa:	d10b      	bne.n	80044c4 <GPIO_DeInit+0x8c>
 80044ac:	f44f 7080 	mov.w	r0, #256	; 0x100
 80044b0:	2101      	movs	r1, #1
 80044b2:	f000 faa7 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 80044b6:	f44f 7080 	mov.w	r0, #256	; 0x100
 80044ba:	2100      	movs	r1, #0
 80044bc:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 80044c0:	f000 baa0 	b.w	8004a04 <RCC_APB2PeriphResetCmd>
 80044c4:	bd10      	pop	{r4, pc}
 80044c6:	bf00      	nop
 80044c8:	40010800 	.word	0x40010800
 80044cc:	40010c00 	.word	0x40010c00
 80044d0:	40011000 	.word	0x40011000
 80044d4:	40011400 	.word	0x40011400
 80044d8:	40011800 	.word	0x40011800
 80044dc:	40011c00 	.word	0x40011c00
 80044e0:	40012000 	.word	0x40012000

080044e4 <GPIO_AFIODeInit>:
 80044e4:	2001      	movs	r0, #1
 80044e6:	4601      	mov	r1, r0
 80044e8:	b510      	push	{r4, lr}
 80044ea:	f000 fa8b 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 80044ee:	2001      	movs	r0, #1
 80044f0:	2100      	movs	r1, #0
 80044f2:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 80044f6:	f000 ba85 	b.w	8004a04 <RCC_APB2PeriphResetCmd>

080044fa <GPIO_Init>:
 80044fa:	78cb      	ldrb	r3, [r1, #3]
 80044fc:	b5f0      	push	{r4, r5, r6, r7, lr}
 80044fe:	f013 0f10 	tst.w	r3, #16
 8004502:	f003 020f 	and.w	r2, r3, #15
 8004506:	bf1c      	itt	ne
 8004508:	788b      	ldrbne	r3, [r1, #2]
 800450a:	431a      	orrne	r2, r3
 800450c:	780b      	ldrb	r3, [r1, #0]
 800450e:	b1e3      	cbz	r3, 800454a <GPIO_Init+0x50>
 8004510:	6804      	ldr	r4, [r0, #0]
 8004512:	2300      	movs	r3, #0
 8004514:	2601      	movs	r6, #1
 8004516:	409e      	lsls	r6, r3
 8004518:	880d      	ldrh	r5, [r1, #0]
 800451a:	ea06 0505 	and.w	r5, r6, r5
 800451e:	42b5      	cmp	r5, r6
 8004520:	d10f      	bne.n	8004542 <GPIO_Init+0x48>
 8004522:	009e      	lsls	r6, r3, #2
 8004524:	270f      	movs	r7, #15
 8004526:	40b7      	lsls	r7, r6
 8004528:	fa12 f606 	lsls.w	r6, r2, r6
 800452c:	ea24 0407 	bic.w	r4, r4, r7
 8004530:	4334      	orrs	r4, r6
 8004532:	78ce      	ldrb	r6, [r1, #3]
 8004534:	2e28      	cmp	r6, #40	; 0x28
 8004536:	d101      	bne.n	800453c <GPIO_Init+0x42>
 8004538:	6145      	str	r5, [r0, #20]
 800453a:	e002      	b.n	8004542 <GPIO_Init+0x48>
 800453c:	2e48      	cmp	r6, #72	; 0x48
 800453e:	bf08      	it	eq
 8004540:	6105      	streq	r5, [r0, #16]
 8004542:	3301      	adds	r3, #1
 8004544:	2b08      	cmp	r3, #8
 8004546:	d1e5      	bne.n	8004514 <GPIO_Init+0x1a>
 8004548:	6004      	str	r4, [r0, #0]
 800454a:	880b      	ldrh	r3, [r1, #0]
 800454c:	2bff      	cmp	r3, #255	; 0xff
 800454e:	d91f      	bls.n	8004590 <GPIO_Init+0x96>
 8004550:	6845      	ldr	r5, [r0, #4]
 8004552:	2300      	movs	r3, #0
 8004554:	2601      	movs	r6, #1
 8004556:	f103 0408 	add.w	r4, r3, #8
 800455a:	fa16 f404 	lsls.w	r4, r6, r4
 800455e:	880e      	ldrh	r6, [r1, #0]
 8004560:	ea04 0606 	and.w	r6, r4, r6
 8004564:	42a6      	cmp	r6, r4
 8004566:	d10f      	bne.n	8004588 <GPIO_Init+0x8e>
 8004568:	009e      	lsls	r6, r3, #2
 800456a:	270f      	movs	r7, #15
 800456c:	40b7      	lsls	r7, r6
 800456e:	fa12 f606 	lsls.w	r6, r2, r6
 8004572:	ea25 0507 	bic.w	r5, r5, r7
 8004576:	4335      	orrs	r5, r6
 8004578:	78ce      	ldrb	r6, [r1, #3]
 800457a:	2e28      	cmp	r6, #40	; 0x28
 800457c:	bf08      	it	eq
 800457e:	6144      	streq	r4, [r0, #20]
 8004580:	78ce      	ldrb	r6, [r1, #3]
 8004582:	2e48      	cmp	r6, #72	; 0x48
 8004584:	bf08      	it	eq
 8004586:	6104      	streq	r4, [r0, #16]
 8004588:	3301      	adds	r3, #1
 800458a:	2b08      	cmp	r3, #8
 800458c:	d1e2      	bne.n	8004554 <GPIO_Init+0x5a>
 800458e:	6045      	str	r5, [r0, #4]
 8004590:	bdf0      	pop	{r4, r5, r6, r7, pc}

08004592 <GPIO_StructInit>:
 8004592:	f04f 33ff 	mov.w	r3, #4294967295
 8004596:	8003      	strh	r3, [r0, #0]
 8004598:	2302      	movs	r3, #2
 800459a:	7083      	strb	r3, [r0, #2]
 800459c:	18db      	adds	r3, r3, r3
 800459e:	70c3      	strb	r3, [r0, #3]
 80045a0:	4770      	bx	lr

080045a2 <GPIO_ReadInputDataBit>:
 80045a2:	6883      	ldr	r3, [r0, #8]
 80045a4:	4219      	tst	r1, r3
 80045a6:	bf0c      	ite	eq
 80045a8:	2000      	moveq	r0, #0
 80045aa:	2001      	movne	r0, #1
 80045ac:	4770      	bx	lr

080045ae <GPIO_ReadInputData>:
 80045ae:	6880      	ldr	r0, [r0, #8]
 80045b0:	b280      	uxth	r0, r0
 80045b2:	4770      	bx	lr

080045b4 <GPIO_ReadOutputDataBit>:
 80045b4:	68c3      	ldr	r3, [r0, #12]
 80045b6:	4219      	tst	r1, r3
 80045b8:	bf0c      	ite	eq
 80045ba:	2000      	moveq	r0, #0
 80045bc:	2001      	movne	r0, #1
 80045be:	4770      	bx	lr

080045c0 <GPIO_ReadOutputData>:
 80045c0:	68c0      	ldr	r0, [r0, #12]
 80045c2:	b280      	uxth	r0, r0
 80045c4:	4770      	bx	lr

080045c6 <GPIO_SetBits>:
 80045c6:	6101      	str	r1, [r0, #16]
 80045c8:	4770      	bx	lr

080045ca <GPIO_ResetBits>:
 80045ca:	6141      	str	r1, [r0, #20]
 80045cc:	4770      	bx	lr

080045ce <GPIO_WriteBit>:
 80045ce:	b10a      	cbz	r2, 80045d4 <GPIO_WriteBit+0x6>
 80045d0:	6101      	str	r1, [r0, #16]
 80045d2:	4770      	bx	lr
 80045d4:	6141      	str	r1, [r0, #20]
 80045d6:	4770      	bx	lr

080045d8 <GPIO_Write>:
 80045d8:	60c1      	str	r1, [r0, #12]
 80045da:	4770      	bx	lr

080045dc <GPIO_PinLockConfig>:
 80045dc:	f441 3380 	orr.w	r3, r1, #65536	; 0x10000
 80045e0:	6183      	str	r3, [r0, #24]
 80045e2:	6181      	str	r1, [r0, #24]
 80045e4:	6183      	str	r3, [r0, #24]
 80045e6:	6983      	ldr	r3, [r0, #24]
 80045e8:	6983      	ldr	r3, [r0, #24]
 80045ea:	4770      	bx	lr

080045ec <GPIO_EventOutputConfig>:
 80045ec:	b510      	push	{r4, lr}
 80045ee:	4a06      	ldr	r2, [pc, #24]	; (8004608 <GPIO_EventOutputConfig+0x1c>)
 80045f0:	f64f 7380 	movw	r3, #65408	; 0xff80
 80045f4:	6814      	ldr	r4, [r2, #0]
 80045f6:	ea41 1100 	orr.w	r1, r1, r0, lsl #4
 80045fa:	ea04 0303 	and.w	r3, r4, r3
 80045fe:	ea41 0303 	orr.w	r3, r1, r3
 8004602:	6013      	str	r3, [r2, #0]
 8004604:	bd10      	pop	{r4, pc}
 8004606:	bf00      	nop
 8004608:	40010000 	.word	0x40010000

0800460c <GPIO_EventOutputCmd>:
 800460c:	4b01      	ldr	r3, [pc, #4]	; (8004614 <GPIO_EventOutputCmd+0x8>)
 800460e:	6018      	str	r0, [r3, #0]
 8004610:	4770      	bx	lr
 8004612:	bf00      	nop
 8004614:	4220001c 	.word	0x4220001c

08004618 <GPIO_PinRemapConfig>:
 8004618:	b530      	push	{r4, r5, lr}
 800461a:	4a14      	ldr	r2, [pc, #80]	; (800466c <GPIO_PinRemapConfig+0x54>)
 800461c:	f400 1540 	and.w	r5, r0, #3145728	; 0x300000
 8004620:	f5b5 1f40 	cmp.w	r5, #3145728	; 0x300000
 8004624:	6853      	ldr	r3, [r2, #4]
 8004626:	b284      	uxth	r4, r0
 8004628:	d106      	bne.n	8004638 <GPIO_PinRemapConfig+0x20>
 800462a:	6855      	ldr	r5, [r2, #4]
 800462c:	f023 6370 	bic.w	r3, r3, #251658240	; 0xf000000
 8004630:	f025 6570 	bic.w	r5, r5, #251658240	; 0xf000000
 8004634:	6055      	str	r5, [r2, #4]
 8004636:	e011      	b.n	800465c <GPIO_PinRemapConfig+0x44>
 8004638:	f410 1f80 	tst.w	r0, #1048576	; 0x100000
 800463c:	d006      	beq.n	800464c <GPIO_PinRemapConfig+0x34>
 800463e:	f400 2270 	and.w	r2, r0, #983040	; 0xf0000
 8004642:	0c12      	lsrs	r2, r2, #16
 8004644:	2503      	movs	r5, #3
 8004646:	fa15 f202 	lsls.w	r2, r5, r2
 800464a:	e003      	b.n	8004654 <GPIO_PinRemapConfig+0x3c>
 800464c:	0d42      	lsrs	r2, r0, #21
 800464e:	0112      	lsls	r2, r2, #4
 8004650:	fa14 f202 	lsls.w	r2, r4, r2
 8004654:	ea23 0302 	bic.w	r3, r3, r2
 8004658:	f043 6370 	orr.w	r3, r3, #251658240	; 0xf000000
 800465c:	b119      	cbz	r1, 8004666 <GPIO_PinRemapConfig+0x4e>
 800465e:	0d40      	lsrs	r0, r0, #21
 8004660:	0100      	lsls	r0, r0, #4
 8004662:	4084      	lsls	r4, r0
 8004664:	4323      	orrs	r3, r4
 8004666:	4a01      	ldr	r2, [pc, #4]	; (800466c <GPIO_PinRemapConfig+0x54>)
 8004668:	6053      	str	r3, [r2, #4]
 800466a:	bd30      	pop	{r4, r5, pc}
 800466c:	40010000 	.word	0x40010000

08004670 <GPIO_EXTILineConfig>:
 8004670:	f001 0203 	and.w	r2, r1, #3
 8004674:	b530      	push	{r4, r5, lr}
 8004676:	0092      	lsls	r2, r2, #2
 8004678:	240f      	movs	r4, #15
 800467a:	4094      	lsls	r4, r2
 800467c:	fa10 f202 	lsls.w	r2, r0, r2
 8004680:	4b07      	ldr	r3, [pc, #28]	; (80046a0 <GPIO_EXTILineConfig+0x30>)
 8004682:	0889      	lsrs	r1, r1, #2
 8004684:	3102      	adds	r1, #2
 8004686:	f853 5021 	ldr.w	r5, [r3, r1, lsl #2]
 800468a:	ea25 0404 	bic.w	r4, r5, r4
 800468e:	f843 4021 	str.w	r4, [r3, r1, lsl #2]
 8004692:	f853 4021 	ldr.w	r4, [r3, r1, lsl #2]
 8004696:	ea42 0404 	orr.w	r4, r2, r4
 800469a:	f843 4021 	str.w	r4, [r3, r1, lsl #2]
 800469e:	bd30      	pop	{r4, r5, pc}
 80046a0:	40010000 	.word	0x40010000

080046a4 <GPIO_ETH_MediaInterfaceConfig>:
 80046a4:	4b01      	ldr	r3, [pc, #4]	; (80046ac <GPIO_ETH_MediaInterfaceConfig+0x8>)
 80046a6:	6018      	str	r0, [r3, #0]
 80046a8:	4770      	bx	lr
 80046aa:	bf00      	nop
 80046ac:	422000dc 	.word	0x422000dc

080046b0 <__WFI>:
 80046b0:	bf30      	wfi
 80046b2:	4770      	bx	lr

080046b4 <PWR_DeInit>:
 80046b4:	2101      	movs	r1, #1
 80046b6:	b510      	push	{r4, lr}
 80046b8:	f04f 5080 	mov.w	r0, #268435456	; 0x10000000
 80046bc:	f000 f9ae 	bl	8004a1c <RCC_APB1PeriphResetCmd>
 80046c0:	f04f 5080 	mov.w	r0, #268435456	; 0x10000000
 80046c4:	2100      	movs	r1, #0
 80046c6:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 80046ca:	f000 b9a7 	b.w	8004a1c <RCC_APB1PeriphResetCmd>

080046ce <PWR_BackupAccessCmd>:
 80046ce:	4b01      	ldr	r3, [pc, #4]	; (80046d4 <PWR_BackupAccessCmd+0x6>)
 80046d0:	6018      	str	r0, [r3, #0]
 80046d2:	4770      	bx	lr
 80046d4:	420e0020 	.word	0x420e0020

080046d8 <PWR_PVDCmd>:
 80046d8:	4b01      	ldr	r3, [pc, #4]	; (80046e0 <PWR_PVDCmd+0x8>)
 80046da:	6018      	str	r0, [r3, #0]
 80046dc:	4770      	bx	lr
 80046de:	bf00      	nop
 80046e0:	420e0010 	.word	0x420e0010

080046e4 <PWR_PVDLevelConfig>:
 80046e4:	4b03      	ldr	r3, [pc, #12]	; (80046f4 <PWR_PVDLevelConfig+0x10>)
 80046e6:	681a      	ldr	r2, [r3, #0]
 80046e8:	f022 02e0 	bic.w	r2, r2, #224	; 0xe0
 80046ec:	ea40 0202 	orr.w	r2, r0, r2
 80046f0:	601a      	str	r2, [r3, #0]
 80046f2:	4770      	bx	lr
 80046f4:	40007000 	.word	0x40007000

080046f8 <PWR_WakeUpPinCmd>:
 80046f8:	4b01      	ldr	r3, [pc, #4]	; (8004700 <PWR_WakeUpPinCmd+0x8>)
 80046fa:	6018      	str	r0, [r3, #0]
 80046fc:	4770      	bx	lr
 80046fe:	bf00      	nop
 8004700:	420e00a0 	.word	0x420e00a0

08004704 <PWR_EnterSTOPMode>:
 8004704:	4b08      	ldr	r3, [pc, #32]	; (8004728 <PWR_EnterSTOPMode+0x24>)
 8004706:	2901      	cmp	r1, #1
 8004708:	681a      	ldr	r2, [r3, #0]
 800470a:	f022 0203 	bic.w	r2, r2, #3
 800470e:	ea40 0202 	orr.w	r2, r0, r2
 8004712:	601a      	str	r2, [r3, #0]
 8004714:	4b05      	ldr	r3, [pc, #20]	; (800472c <PWR_EnterSTOPMode+0x28>)
 8004716:	681a      	ldr	r2, [r3, #0]
 8004718:	f042 0204 	orr.w	r2, r2, #4
 800471c:	601a      	str	r2, [r3, #0]
 800471e:	d100      	bne.n	8004722 <PWR_EnterSTOPMode+0x1e>
 8004720:	e7c6      	b.n	80046b0 <__WFI>
 8004722:	bf20      	wfe
 8004724:	4770      	bx	lr
 8004726:	bf00      	nop
 8004728:	40007000 	.word	0x40007000
 800472c:	e000ed10 	.word	0xe000ed10

08004730 <PWR_EnterSTANDBYMode>:
 8004730:	4b07      	ldr	r3, [pc, #28]	; (8004750 <PWR_EnterSTANDBYMode+0x20>)
 8004732:	681a      	ldr	r2, [r3, #0]
 8004734:	f042 0204 	orr.w	r2, r2, #4
 8004738:	601a      	str	r2, [r3, #0]
 800473a:	681a      	ldr	r2, [r3, #0]
 800473c:	f042 0202 	orr.w	r2, r2, #2
 8004740:	601a      	str	r2, [r3, #0]
 8004742:	4b04      	ldr	r3, [pc, #16]	; (8004754 <PWR_EnterSTANDBYMode+0x24>)
 8004744:	681a      	ldr	r2, [r3, #0]
 8004746:	f042 0204 	orr.w	r2, r2, #4
 800474a:	601a      	str	r2, [r3, #0]
 800474c:	e7b0      	b.n	80046b0 <__WFI>
 800474e:	bf00      	nop
 8004750:	40007000 	.word	0x40007000
 8004754:	e000ed10 	.word	0xe000ed10

08004758 <PWR_GetFlagStatus>:
 8004758:	4b03      	ldr	r3, [pc, #12]	; (8004768 <PWR_GetFlagStatus+0x10>)
 800475a:	685b      	ldr	r3, [r3, #4]
 800475c:	4218      	tst	r0, r3
 800475e:	bf0c      	ite	eq
 8004760:	2000      	moveq	r0, #0
 8004762:	2001      	movne	r0, #1
 8004764:	4770      	bx	lr
 8004766:	bf00      	nop
 8004768:	40007000 	.word	0x40007000

0800476c <PWR_ClearFlag>:
 800476c:	4b02      	ldr	r3, [pc, #8]	; (8004778 <PWR_ClearFlag+0xc>)
 800476e:	681a      	ldr	r2, [r3, #0]
 8004770:	ea42 0280 	orr.w	r2, r2, r0, lsl #2
 8004774:	601a      	str	r2, [r3, #0]
 8004776:	4770      	bx	lr
 8004778:	40007000 	.word	0x40007000

0800477c <RCC_DeInit>:
 800477c:	4b0d      	ldr	r3, [pc, #52]	; (80047b4 <RCC_DeInit+0x38>)
 800477e:	681a      	ldr	r2, [r3, #0]
 8004780:	f042 0201 	orr.w	r2, r2, #1
 8004784:	601a      	str	r2, [r3, #0]
 8004786:	6859      	ldr	r1, [r3, #4]
 8004788:	4a0b      	ldr	r2, [pc, #44]	; (80047b8 <RCC_DeInit+0x3c>)
 800478a:	ea01 0202 	and.w	r2, r1, r2
 800478e:	605a      	str	r2, [r3, #4]
 8004790:	681a      	ldr	r2, [r3, #0]
 8004792:	f022 7284 	bic.w	r2, r2, #17301504	; 0x1080000
 8004796:	f422 3280 	bic.w	r2, r2, #65536	; 0x10000
 800479a:	601a      	str	r2, [r3, #0]
 800479c:	681a      	ldr	r2, [r3, #0]
 800479e:	f422 2280 	bic.w	r2, r2, #262144	; 0x40000
 80047a2:	601a      	str	r2, [r3, #0]
 80047a4:	685a      	ldr	r2, [r3, #4]
 80047a6:	f422 02fe 	bic.w	r2, r2, #8323072	; 0x7f0000
 80047aa:	605a      	str	r2, [r3, #4]
 80047ac:	f44f 021f 	mov.w	r2, #10420224	; 0x9f0000
 80047b0:	609a      	str	r2, [r3, #8]
 80047b2:	4770      	bx	lr
 80047b4:	40021000 	.word	0x40021000
 80047b8:	f8ff0000 	.word	0xf8ff0000

080047bc <RCC_HSEConfig>:
 80047bc:	4b0c      	ldr	r3, [pc, #48]	; (80047f0 <RCC_HSEConfig+0x34>)
 80047be:	f5b0 3f80 	cmp.w	r0, #65536	; 0x10000
 80047c2:	681a      	ldr	r2, [r3, #0]
 80047c4:	f422 3280 	bic.w	r2, r2, #65536	; 0x10000
 80047c8:	601a      	str	r2, [r3, #0]
 80047ca:	681a      	ldr	r2, [r3, #0]
 80047cc:	f422 2280 	bic.w	r2, r2, #262144	; 0x40000
 80047d0:	601a      	str	r2, [r3, #0]
 80047d2:	d003      	beq.n	80047dc <RCC_HSEConfig+0x20>
 80047d4:	f5b0 2f80 	cmp.w	r0, #262144	; 0x40000
 80047d8:	d109      	bne.n	80047ee <RCC_HSEConfig+0x32>
 80047da:	e004      	b.n	80047e6 <RCC_HSEConfig+0x2a>
 80047dc:	681a      	ldr	r2, [r3, #0]
 80047de:	f442 3280 	orr.w	r2, r2, #65536	; 0x10000
 80047e2:	601a      	str	r2, [r3, #0]
 80047e4:	4770      	bx	lr
 80047e6:	681a      	ldr	r2, [r3, #0]
 80047e8:	f442 22a0 	orr.w	r2, r2, #327680	; 0x50000
 80047ec:	601a      	str	r2, [r3, #0]
 80047ee:	4770      	bx	lr
 80047f0:	40021000 	.word	0x40021000

080047f4 <RCC_AdjustHSICalibrationValue>:
 80047f4:	4b03      	ldr	r3, [pc, #12]	; (8004804 <RCC_AdjustHSICalibrationValue+0x10>)
 80047f6:	681a      	ldr	r2, [r3, #0]
 80047f8:	f022 02f8 	bic.w	r2, r2, #248	; 0xf8
 80047fc:	ea42 02c0 	orr.w	r2, r2, r0, lsl #3
 8004800:	601a      	str	r2, [r3, #0]
 8004802:	4770      	bx	lr
 8004804:	40021000 	.word	0x40021000

08004808 <RCC_HSICmd>:
 8004808:	4b01      	ldr	r3, [pc, #4]	; (8004810 <RCC_HSICmd+0x8>)
 800480a:	6018      	str	r0, [r3, #0]
 800480c:	4770      	bx	lr
 800480e:	bf00      	nop
 8004810:	42420000 	.word	0x42420000

08004814 <RCC_PLLConfig>:
 8004814:	4b04      	ldr	r3, [pc, #16]	; (8004828 <RCC_PLLConfig+0x14>)
 8004816:	685a      	ldr	r2, [r3, #4]
 8004818:	f422 127c 	bic.w	r2, r2, #4128768	; 0x3f0000
 800481c:	ea40 0202 	orr.w	r2, r0, r2
 8004820:	430a      	orrs	r2, r1
 8004822:	605a      	str	r2, [r3, #4]
 8004824:	4770      	bx	lr
 8004826:	bf00      	nop
 8004828:	40021000 	.word	0x40021000

0800482c <RCC_PLLCmd>:
 800482c:	4b01      	ldr	r3, [pc, #4]	; (8004834 <RCC_PLLCmd+0x8>)
 800482e:	6018      	str	r0, [r3, #0]
 8004830:	4770      	bx	lr
 8004832:	bf00      	nop
 8004834:	42420060 	.word	0x42420060

08004838 <RCC_SYSCLKConfig>:
 8004838:	4b03      	ldr	r3, [pc, #12]	; (8004848 <RCC_SYSCLKConfig+0x10>)
 800483a:	685a      	ldr	r2, [r3, #4]
 800483c:	f022 0203 	bic.w	r2, r2, #3
 8004840:	ea40 0202 	orr.w	r2, r0, r2
 8004844:	605a      	str	r2, [r3, #4]
 8004846:	4770      	bx	lr
 8004848:	40021000 	.word	0x40021000

0800484c <RCC_GetSYSCLKSource>:
 800484c:	4b02      	ldr	r3, [pc, #8]	; (8004858 <RCC_GetSYSCLKSource+0xc>)
 800484e:	6858      	ldr	r0, [r3, #4]
 8004850:	f000 000c 	and.w	r0, r0, #12
 8004854:	4770      	bx	lr
 8004856:	bf00      	nop
 8004858:	40021000 	.word	0x40021000

0800485c <RCC_HCLKConfig>:
 800485c:	4b03      	ldr	r3, [pc, #12]	; (800486c <RCC_HCLKConfig+0x10>)
 800485e:	685a      	ldr	r2, [r3, #4]
 8004860:	f022 02f0 	bic.w	r2, r2, #240	; 0xf0
 8004864:	ea40 0202 	orr.w	r2, r0, r2
 8004868:	605a      	str	r2, [r3, #4]
 800486a:	4770      	bx	lr
 800486c:	40021000 	.word	0x40021000

08004870 <RCC_PCLK1Config>:
 8004870:	4b03      	ldr	r3, [pc, #12]	; (8004880 <RCC_PCLK1Config+0x10>)
 8004872:	685a      	ldr	r2, [r3, #4]
 8004874:	f422 62e0 	bic.w	r2, r2, #1792	; 0x700
 8004878:	ea40 0202 	orr.w	r2, r0, r2
 800487c:	605a      	str	r2, [r3, #4]
 800487e:	4770      	bx	lr
 8004880:	40021000 	.word	0x40021000

08004884 <RCC_PCLK2Config>:
 8004884:	4b03      	ldr	r3, [pc, #12]	; (8004894 <RCC_PCLK2Config+0x10>)
 8004886:	685a      	ldr	r2, [r3, #4]
 8004888:	f422 5260 	bic.w	r2, r2, #14336	; 0x3800
 800488c:	ea42 02c0 	orr.w	r2, r2, r0, lsl #3
 8004890:	605a      	str	r2, [r3, #4]
 8004892:	4770      	bx	lr
 8004894:	40021000 	.word	0x40021000

08004898 <RCC_ITConfig>:
 8004898:	4b04      	ldr	r3, [pc, #16]	; (80048ac <RCC_ITConfig+0x14>)
 800489a:	b111      	cbz	r1, 80048a2 <RCC_ITConfig+0xa>
 800489c:	781a      	ldrb	r2, [r3, #0]
 800489e:	4310      	orrs	r0, r2
 80048a0:	e002      	b.n	80048a8 <RCC_ITConfig+0x10>
 80048a2:	781a      	ldrb	r2, [r3, #0]
 80048a4:	ea22 0000 	bic.w	r0, r2, r0
 80048a8:	7018      	strb	r0, [r3, #0]
 80048aa:	4770      	bx	lr
 80048ac:	40021009 	.word	0x40021009

080048b0 <RCC_USBCLKConfig>:
 80048b0:	4b01      	ldr	r3, [pc, #4]	; (80048b8 <RCC_USBCLKConfig+0x8>)
 80048b2:	6018      	str	r0, [r3, #0]
 80048b4:	4770      	bx	lr
 80048b6:	bf00      	nop
 80048b8:	424200d8 	.word	0x424200d8

080048bc <RCC_ADCCLKConfig>:
 80048bc:	4b03      	ldr	r3, [pc, #12]	; (80048cc <RCC_ADCCLKConfig+0x10>)
 80048be:	685a      	ldr	r2, [r3, #4]
 80048c0:	f422 4240 	bic.w	r2, r2, #49152	; 0xc000
 80048c4:	ea40 0202 	orr.w	r2, r0, r2
 80048c8:	605a      	str	r2, [r3, #4]
 80048ca:	4770      	bx	lr
 80048cc:	40021000 	.word	0x40021000

080048d0 <RCC_LSEConfig>:
 80048d0:	4b06      	ldr	r3, [pc, #24]	; (80048ec <RCC_LSEConfig+0x1c>)
 80048d2:	2200      	movs	r2, #0
 80048d4:	2801      	cmp	r0, #1
 80048d6:	701a      	strb	r2, [r3, #0]
 80048d8:	701a      	strb	r2, [r3, #0]
 80048da:	d002      	beq.n	80048e2 <RCC_LSEConfig+0x12>
 80048dc:	2804      	cmp	r0, #4
 80048de:	d104      	bne.n	80048ea <RCC_LSEConfig+0x1a>
 80048e0:	e001      	b.n	80048e6 <RCC_LSEConfig+0x16>
 80048e2:	7018      	strb	r0, [r3, #0]
 80048e4:	4770      	bx	lr
 80048e6:	2205      	movs	r2, #5
 80048e8:	701a      	strb	r2, [r3, #0]
 80048ea:	4770      	bx	lr
 80048ec:	40021020 	.word	0x40021020

080048f0 <RCC_LSICmd>:
 80048f0:	4b01      	ldr	r3, [pc, #4]	; (80048f8 <RCC_LSICmd+0x8>)
 80048f2:	6018      	str	r0, [r3, #0]
 80048f4:	4770      	bx	lr
 80048f6:	bf00      	nop
 80048f8:	42420480 	.word	0x42420480

080048fc <RCC_RTCCLKConfig>:
 80048fc:	4b02      	ldr	r3, [pc, #8]	; (8004908 <RCC_RTCCLKConfig+0xc>)
 80048fe:	6a1a      	ldr	r2, [r3, #32]
 8004900:	ea40 0202 	orr.w	r2, r0, r2
 8004904:	621a      	str	r2, [r3, #32]
 8004906:	4770      	bx	lr
 8004908:	40021000 	.word	0x40021000

0800490c <RCC_RTCCLKCmd>:
 800490c:	4b01      	ldr	r3, [pc, #4]	; (8004914 <RCC_RTCCLKCmd+0x8>)
 800490e:	6018      	str	r0, [r3, #0]
 8004910:	4770      	bx	lr
 8004912:	bf00      	nop
 8004914:	4242043c 	.word	0x4242043c

08004918 <RCC_GetClocksFreq>:
 8004918:	b510      	push	{r4, lr}
 800491a:	4b22      	ldr	r3, [pc, #136]	; (80049a4 <RCC_GetClocksFreq+0x8c>)
 800491c:	685a      	ldr	r2, [r3, #4]
 800491e:	f002 020c 	and.w	r2, r2, #12
 8004922:	2a04      	cmp	r2, #4
 8004924:	d003      	beq.n	800492e <RCC_GetClocksFreq+0x16>
 8004926:	2a08      	cmp	r2, #8
 8004928:	d004      	beq.n	8004934 <RCC_GetClocksFreq+0x1c>
 800492a:	4b1f      	ldr	r3, [pc, #124]	; (80049a8 <RCC_GetClocksFreq+0x90>)
 800492c:	e000      	b.n	8004930 <RCC_GetClocksFreq+0x18>
 800492e:	4b1f      	ldr	r3, [pc, #124]	; (80049ac <RCC_GetClocksFreq+0x94>)
 8004930:	6003      	str	r3, [r0, #0]
 8004932:	e012      	b.n	800495a <RCC_GetClocksFreq+0x42>
 8004934:	685a      	ldr	r2, [r3, #4]
 8004936:	6859      	ldr	r1, [r3, #4]
 8004938:	f402 1270 	and.w	r2, r2, #3932160	; 0x3c0000
 800493c:	0c92      	lsrs	r2, r2, #18
 800493e:	3202      	adds	r2, #2
 8004940:	f411 3f80 	tst.w	r1, #65536	; 0x10000
 8004944:	d101      	bne.n	800494a <RCC_GetClocksFreq+0x32>
 8004946:	4b1a      	ldr	r3, [pc, #104]	; (80049b0 <RCC_GetClocksFreq+0x98>)
 8004948:	e005      	b.n	8004956 <RCC_GetClocksFreq+0x3e>
 800494a:	685b      	ldr	r3, [r3, #4]
 800494c:	f413 3f00 	tst.w	r3, #131072	; 0x20000
 8004950:	bf14      	ite	ne
 8004952:	4b18      	ldrne	r3, [pc, #96]	; (80049b4 <RCC_GetClocksFreq+0x9c>)
 8004954:	4b15      	ldreq	r3, [pc, #84]	; (80049ac <RCC_GetClocksFreq+0x94>)
 8004956:	435a      	muls	r2, r3
 8004958:	6002      	str	r2, [r0, #0]
 800495a:	4a12      	ldr	r2, [pc, #72]	; (80049a4 <RCC_GetClocksFreq+0x8c>)
 800495c:	4b16      	ldr	r3, [pc, #88]	; (80049b8 <RCC_GetClocksFreq+0xa0>)
 800495e:	6851      	ldr	r1, [r2, #4]
 8004960:	6804      	ldr	r4, [r0, #0]
 8004962:	f001 01f0 	and.w	r1, r1, #240	; 0xf0
 8004966:	0909      	lsrs	r1, r1, #4
 8004968:	5c59      	ldrb	r1, [r3, r1]
 800496a:	fa34 f101 	lsrs.w	r1, r4, r1
 800496e:	6041      	str	r1, [r0, #4]
 8004970:	6854      	ldr	r4, [r2, #4]
 8004972:	f404 64e0 	and.w	r4, r4, #1792	; 0x700
 8004976:	0a24      	lsrs	r4, r4, #8
 8004978:	5d1c      	ldrb	r4, [r3, r4]
 800497a:	fa31 f404 	lsrs.w	r4, r1, r4
 800497e:	6084      	str	r4, [r0, #8]
 8004980:	6854      	ldr	r4, [r2, #4]
 8004982:	f404 5460 	and.w	r4, r4, #14336	; 0x3800
 8004986:	0ae4      	lsrs	r4, r4, #11
 8004988:	5d1c      	ldrb	r4, [r3, r4]
 800498a:	40e1      	lsrs	r1, r4
 800498c:	60c1      	str	r1, [r0, #12]
 800498e:	6852      	ldr	r2, [r2, #4]
 8004990:	f402 4240 	and.w	r2, r2, #49152	; 0xc000
 8004994:	eb03 3392 	add.w	r3, r3, r2, lsr #14
 8004998:	7c1b      	ldrb	r3, [r3, #16]
 800499a:	fbb1 f1f3 	udiv	r1, r1, r3
 800499e:	6101      	str	r1, [r0, #16]
 80049a0:	bd10      	pop	{r4, pc}
 80049a2:	bf00      	nop
 80049a4:	40021000 	.word	0x40021000
 80049a8:	007a1200 	.word	0x007a1200
 80049ac:	00b71b00 	.word	0x00b71b00
 80049b0:	003d0900 	.word	0x003d0900
 80049b4:	005b8d80 	.word	0x005b8d80
 80049b8:	2000015c 	.word	0x2000015c

080049bc <RCC_AHBPeriphClockCmd>:
 80049bc:	4b04      	ldr	r3, [pc, #16]	; (80049d0 <RCC_AHBPeriphClockCmd+0x14>)
 80049be:	b111      	cbz	r1, 80049c6 <RCC_AHBPeriphClockCmd+0xa>
 80049c0:	695a      	ldr	r2, [r3, #20]
 80049c2:	4310      	orrs	r0, r2
 80049c4:	e002      	b.n	80049cc <RCC_AHBPeriphClockCmd+0x10>
 80049c6:	695a      	ldr	r2, [r3, #20]
 80049c8:	ea22 0000 	bic.w	r0, r2, r0
 80049cc:	6158      	str	r0, [r3, #20]
 80049ce:	4770      	bx	lr
 80049d0:	40021000 	.word	0x40021000

080049d4 <RCC_APB2PeriphClockCmd>:
 80049d4:	4b04      	ldr	r3, [pc, #16]	; (80049e8 <RCC_APB2PeriphClockCmd+0x14>)
 80049d6:	b111      	cbz	r1, 80049de <RCC_APB2PeriphClockCmd+0xa>
 80049d8:	699a      	ldr	r2, [r3, #24]
 80049da:	4310      	orrs	r0, r2
 80049dc:	e002      	b.n	80049e4 <RCC_APB2PeriphClockCmd+0x10>
 80049de:	699a      	ldr	r2, [r3, #24]
 80049e0:	ea22 0000 	bic.w	r0, r2, r0
 80049e4:	6198      	str	r0, [r3, #24]
 80049e6:	4770      	bx	lr
 80049e8:	40021000 	.word	0x40021000

080049ec <RCC_APB1PeriphClockCmd>:
 80049ec:	4b04      	ldr	r3, [pc, #16]	; (8004a00 <RCC_APB1PeriphClockCmd+0x14>)
 80049ee:	b111      	cbz	r1, 80049f6 <RCC_APB1PeriphClockCmd+0xa>
 80049f0:	69da      	ldr	r2, [r3, #28]
 80049f2:	4310      	orrs	r0, r2
 80049f4:	e002      	b.n	80049fc <RCC_APB1PeriphClockCmd+0x10>
 80049f6:	69da      	ldr	r2, [r3, #28]
 80049f8:	ea22 0000 	bic.w	r0, r2, r0
 80049fc:	61d8      	str	r0, [r3, #28]
 80049fe:	4770      	bx	lr
 8004a00:	40021000 	.word	0x40021000

08004a04 <RCC_APB2PeriphResetCmd>:
 8004a04:	4b04      	ldr	r3, [pc, #16]	; (8004a18 <RCC_APB2PeriphResetCmd+0x14>)
 8004a06:	b111      	cbz	r1, 8004a0e <RCC_APB2PeriphResetCmd+0xa>
 8004a08:	68da      	ldr	r2, [r3, #12]
 8004a0a:	4310      	orrs	r0, r2
 8004a0c:	e002      	b.n	8004a14 <RCC_APB2PeriphResetCmd+0x10>
 8004a0e:	68da      	ldr	r2, [r3, #12]
 8004a10:	ea22 0000 	bic.w	r0, r2, r0
 8004a14:	60d8      	str	r0, [r3, #12]
 8004a16:	4770      	bx	lr
 8004a18:	40021000 	.word	0x40021000

08004a1c <RCC_APB1PeriphResetCmd>:
 8004a1c:	4b04      	ldr	r3, [pc, #16]	; (8004a30 <RCC_APB1PeriphResetCmd+0x14>)
 8004a1e:	b111      	cbz	r1, 8004a26 <RCC_APB1PeriphResetCmd+0xa>
 8004a20:	691a      	ldr	r2, [r3, #16]
 8004a22:	4310      	orrs	r0, r2
 8004a24:	e002      	b.n	8004a2c <RCC_APB1PeriphResetCmd+0x10>
 8004a26:	691a      	ldr	r2, [r3, #16]
 8004a28:	ea22 0000 	bic.w	r0, r2, r0
 8004a2c:	6118      	str	r0, [r3, #16]
 8004a2e:	4770      	bx	lr
 8004a30:	40021000 	.word	0x40021000

08004a34 <RCC_BackupResetCmd>:
 8004a34:	4b01      	ldr	r3, [pc, #4]	; (8004a3c <RCC_BackupResetCmd+0x8>)
 8004a36:	6018      	str	r0, [r3, #0]
 8004a38:	4770      	bx	lr
 8004a3a:	bf00      	nop
 8004a3c:	42420440 	.word	0x42420440

08004a40 <RCC_ClockSecuritySystemCmd>:
 8004a40:	4b01      	ldr	r3, [pc, #4]	; (8004a48 <RCC_ClockSecuritySystemCmd+0x8>)
 8004a42:	6018      	str	r0, [r3, #0]
 8004a44:	4770      	bx	lr
 8004a46:	bf00      	nop
 8004a48:	4242004c 	.word	0x4242004c

08004a4c <RCC_MCOConfig>:
 8004a4c:	4b01      	ldr	r3, [pc, #4]	; (8004a54 <RCC_MCOConfig+0x8>)
 8004a4e:	7018      	strb	r0, [r3, #0]
 8004a50:	4770      	bx	lr
 8004a52:	bf00      	nop
 8004a54:	40021007 	.word	0x40021007

08004a58 <RCC_GetFlagStatus>:
 8004a58:	0943      	lsrs	r3, r0, #5
 8004a5a:	2b01      	cmp	r3, #1
 8004a5c:	4a07      	ldr	r2, [pc, #28]	; (8004a7c <RCC_GetFlagStatus+0x24>)
 8004a5e:	d101      	bne.n	8004a64 <RCC_GetFlagStatus+0xc>
 8004a60:	6813      	ldr	r3, [r2, #0]
 8004a62:	e003      	b.n	8004a6c <RCC_GetFlagStatus+0x14>
 8004a64:	2b02      	cmp	r3, #2
 8004a66:	bf0c      	ite	eq
 8004a68:	6a13      	ldreq	r3, [r2, #32]
 8004a6a:	6a53      	ldrne	r3, [r2, #36]	; 0x24
 8004a6c:	f000 001f 	and.w	r0, r0, #31
 8004a70:	fa33 f000 	lsrs.w	r0, r3, r0
 8004a74:	f000 0001 	and.w	r0, r0, #1
 8004a78:	4770      	bx	lr
 8004a7a:	bf00      	nop
 8004a7c:	40021000 	.word	0x40021000

08004a80 <RCC_WaitForHSEStartUp>:
 8004a80:	b507      	push	{r0, r1, r2, lr}
 8004a82:	2300      	movs	r3, #0
 8004a84:	9301      	str	r3, [sp, #4]
 8004a86:	2031      	movs	r0, #49	; 0x31
 8004a88:	f7ff ffe6 	bl	8004a58 <RCC_GetFlagStatus>
 8004a8c:	9b01      	ldr	r3, [sp, #4]
 8004a8e:	3301      	adds	r3, #1
 8004a90:	9301      	str	r3, [sp, #4]
 8004a92:	9b01      	ldr	r3, [sp, #4]
 8004a94:	f5b3 6fa0 	cmp.w	r3, #1280	; 0x500
 8004a98:	d001      	beq.n	8004a9e <RCC_WaitForHSEStartUp+0x1e>
 8004a9a:	2800      	cmp	r0, #0
 8004a9c:	d0f3      	beq.n	8004a86 <RCC_WaitForHSEStartUp+0x6>
 8004a9e:	2031      	movs	r0, #49	; 0x31
 8004aa0:	f7ff ffda 	bl	8004a58 <RCC_GetFlagStatus>
 8004aa4:	3800      	subs	r0, #0
 8004aa6:	bf18      	it	ne
 8004aa8:	2001      	movne	r0, #1
 8004aaa:	bd0e      	pop	{r1, r2, r3, pc}

08004aac <RCC_ClearFlag>:
 8004aac:	4b02      	ldr	r3, [pc, #8]	; (8004ab8 <RCC_ClearFlag+0xc>)
 8004aae:	6a5a      	ldr	r2, [r3, #36]	; 0x24
 8004ab0:	f042 7280 	orr.w	r2, r2, #16777216	; 0x1000000
 8004ab4:	625a      	str	r2, [r3, #36]	; 0x24
 8004ab6:	4770      	bx	lr
 8004ab8:	40021000 	.word	0x40021000

08004abc <RCC_GetITStatus>:
 8004abc:	4b03      	ldr	r3, [pc, #12]	; (8004acc <RCC_GetITStatus+0x10>)
 8004abe:	689b      	ldr	r3, [r3, #8]
 8004ac0:	4218      	tst	r0, r3
 8004ac2:	bf0c      	ite	eq
 8004ac4:	2000      	moveq	r0, #0
 8004ac6:	2001      	movne	r0, #1
 8004ac8:	4770      	bx	lr
 8004aca:	bf00      	nop
 8004acc:	40021000 	.word	0x40021000

08004ad0 <RCC_ClearITPendingBit>:
 8004ad0:	4b01      	ldr	r3, [pc, #4]	; (8004ad8 <RCC_ClearITPendingBit+0x8>)
 8004ad2:	7018      	strb	r0, [r3, #0]
 8004ad4:	4770      	bx	lr
 8004ad6:	bf00      	nop
 8004ad8:	4002100a 	.word	0x4002100a

08004adc <SPI_I2S_DeInit>:
 8004adc:	4b15      	ldr	r3, [pc, #84]	; (8004b34 <SPI_I2S_DeInit+0x58>)
 8004ade:	b510      	push	{r4, lr}
 8004ae0:	4298      	cmp	r0, r3
 8004ae2:	d10b      	bne.n	8004afc <SPI_I2S_DeInit+0x20>
 8004ae4:	2101      	movs	r1, #1
 8004ae6:	f44f 5080 	mov.w	r0, #4096	; 0x1000
 8004aea:	f7ff ff8b 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 8004aee:	f44f 5080 	mov.w	r0, #4096	; 0x1000
 8004af2:	2100      	movs	r1, #0
 8004af4:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 8004af8:	f7ff bf84 	b.w	8004a04 <RCC_APB2PeriphResetCmd>
 8004afc:	4b0e      	ldr	r3, [pc, #56]	; (8004b38 <SPI_I2S_DeInit+0x5c>)
 8004afe:	4298      	cmp	r0, r3
 8004b00:	d107      	bne.n	8004b12 <SPI_I2S_DeInit+0x36>
 8004b02:	f44f 4080 	mov.w	r0, #16384	; 0x4000
 8004b06:	2101      	movs	r1, #1
 8004b08:	f7ff ff88 	bl	8004a1c <RCC_APB1PeriphResetCmd>
 8004b0c:	f44f 4080 	mov.w	r0, #16384	; 0x4000
 8004b10:	e009      	b.n	8004b26 <SPI_I2S_DeInit+0x4a>
 8004b12:	4b0a      	ldr	r3, [pc, #40]	; (8004b3c <SPI_I2S_DeInit+0x60>)
 8004b14:	4298      	cmp	r0, r3
 8004b16:	d10b      	bne.n	8004b30 <SPI_I2S_DeInit+0x54>
 8004b18:	f44f 4000 	mov.w	r0, #32768	; 0x8000
 8004b1c:	2101      	movs	r1, #1
 8004b1e:	f7ff ff7d 	bl	8004a1c <RCC_APB1PeriphResetCmd>
 8004b22:	f44f 4000 	mov.w	r0, #32768	; 0x8000
 8004b26:	2100      	movs	r1, #0
 8004b28:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 8004b2c:	f7ff bf76 	b.w	8004a1c <RCC_APB1PeriphResetCmd>
 8004b30:	bd10      	pop	{r4, pc}
 8004b32:	bf00      	nop
 8004b34:	40013000 	.word	0x40013000
 8004b38:	40003800 	.word	0x40003800
 8004b3c:	40003c00 	.word	0x40003c00

08004b40 <SPI_Init>:
 8004b40:	880b      	ldrh	r3, [r1, #0]
 8004b42:	b510      	push	{r4, lr}
 8004b44:	884c      	ldrh	r4, [r1, #2]
 8004b46:	8802      	ldrh	r2, [r0, #0]
 8004b48:	ea44 0303 	orr.w	r3, r4, r3
 8004b4c:	888c      	ldrh	r4, [r1, #4]
 8004b4e:	f402 5241 	and.w	r2, r2, #12352	; 0x3040
 8004b52:	4323      	orrs	r3, r4
 8004b54:	88cc      	ldrh	r4, [r1, #6]
 8004b56:	4323      	orrs	r3, r4
 8004b58:	890c      	ldrh	r4, [r1, #8]
 8004b5a:	4323      	orrs	r3, r4
 8004b5c:	894c      	ldrh	r4, [r1, #10]
 8004b5e:	4323      	orrs	r3, r4
 8004b60:	898c      	ldrh	r4, [r1, #12]
 8004b62:	4323      	orrs	r3, r4
 8004b64:	89cc      	ldrh	r4, [r1, #14]
 8004b66:	4323      	orrs	r3, r4
 8004b68:	ea42 0303 	orr.w	r3, r2, r3
 8004b6c:	8003      	strh	r3, [r0, #0]
 8004b6e:	8b83      	ldrh	r3, [r0, #28]
 8004b70:	f423 6300 	bic.w	r3, r3, #2048	; 0x800
 8004b74:	041b      	lsls	r3, r3, #16
 8004b76:	0c1b      	lsrs	r3, r3, #16
 8004b78:	8383      	strh	r3, [r0, #28]
 8004b7a:	8a0b      	ldrh	r3, [r1, #16]
 8004b7c:	8203      	strh	r3, [r0, #16]
 8004b7e:	bd10      	pop	{r4, pc}

08004b80 <I2S_Init>:
 8004b80:	b5f0      	push	{r4, r5, r6, r7, lr}
 8004b82:	8b83      	ldrh	r3, [r0, #28]
 8004b84:	b087      	sub	sp, #28
 8004b86:	f423 637a 	bic.w	r3, r3, #4000	; 0xfa0
 8004b8a:	f023 031f 	bic.w	r3, r3, #31
 8004b8e:	041b      	lsls	r3, r3, #16
 8004b90:	0c1b      	lsrs	r3, r3, #16
 8004b92:	8383      	strh	r3, [r0, #28]
 8004b94:	2302      	movs	r3, #2
 8004b96:	8403      	strh	r3, [r0, #32]
 8004b98:	890b      	ldrh	r3, [r1, #8]
 8004b9a:	4604      	mov	r4, r0
 8004b9c:	2b02      	cmp	r3, #2
 8004b9e:	460d      	mov	r5, r1
 8004ba0:	8b86      	ldrh	r6, [r0, #28]
 8004ba2:	d023      	beq.n	8004bec <I2S_Init+0x6c>
 8004ba4:	a801      	add	r0, sp, #4
 8004ba6:	888f      	ldrh	r7, [r1, #4]
 8004ba8:	f7ff feb6 	bl	8004918 <RCC_GetClocksFreq>
 8004bac:	88eb      	ldrh	r3, [r5, #6]
 8004bae:	2f00      	cmp	r7, #0
 8004bb0:	bf14      	ite	ne
 8004bb2:	2702      	movne	r7, #2
 8004bb4:	2701      	moveq	r7, #1
 8004bb6:	f5b3 7f00 	cmp.w	r3, #512	; 0x200
 8004bba:	9a01      	ldr	r2, [sp, #4]
 8004bbc:	f04f 030a 	mov.w	r3, #10
 8004bc0:	d101      	bne.n	8004bc6 <I2S_Init+0x46>
 8004bc2:	0a12      	lsrs	r2, r2, #8
 8004bc4:	e002      	b.n	8004bcc <I2S_Init+0x4c>
 8004bc6:	017f      	lsls	r7, r7, #5
 8004bc8:	fbb2 f2f7 	udiv	r2, r2, r7
 8004bcc:	4353      	muls	r3, r2
 8004bce:	8929      	ldrh	r1, [r5, #8]
 8004bd0:	220a      	movs	r2, #10
 8004bd2:	fbb3 f3f1 	udiv	r3, r3, r1
 8004bd6:	3305      	adds	r3, #5
 8004bd8:	b29b      	uxth	r3, r3
 8004bda:	fbb3 f3f2 	udiv	r3, r3, r2
 8004bde:	f003 0201 	and.w	r2, r3, #1
 8004be2:	0212      	lsls	r2, r2, #8
 8004be4:	085b      	lsrs	r3, r3, #1
 8004be6:	b29b      	uxth	r3, r3
 8004be8:	b292      	uxth	r2, r2
 8004bea:	e000      	b.n	8004bee <I2S_Init+0x6e>
 8004bec:	2200      	movs	r2, #0
 8004bee:	1e99      	subs	r1, r3, #2
 8004bf0:	b289      	uxth	r1, r1
 8004bf2:	29fe      	cmp	r1, #254	; 0xfe
 8004bf4:	bf28      	it	cs
 8004bf6:	2200      	movcs	r2, #0
 8004bf8:	88e9      	ldrh	r1, [r5, #6]
 8004bfa:	bf28      	it	cs
 8004bfc:	2302      	movcs	r3, #2
 8004bfe:	4313      	orrs	r3, r2
 8004c00:	430b      	orrs	r3, r1
 8004c02:	8423      	strh	r3, [r4, #32]
 8004c04:	882b      	ldrh	r3, [r5, #0]
 8004c06:	886a      	ldrh	r2, [r5, #2]
 8004c08:	ea46 0303 	orr.w	r3, r6, r3
 8004c0c:	f443 6300 	orr.w	r3, r3, #2048	; 0x800
 8004c10:	4313      	orrs	r3, r2
 8004c12:	88aa      	ldrh	r2, [r5, #4]
 8004c14:	4313      	orrs	r3, r2
 8004c16:	896a      	ldrh	r2, [r5, #10]
 8004c18:	4313      	orrs	r3, r2
 8004c1a:	83a3      	strh	r3, [r4, #28]
 8004c1c:	b007      	add	sp, #28
 8004c1e:	bdf0      	pop	{r4, r5, r6, r7, pc}

08004c20 <SPI_StructInit>:
 8004c20:	2300      	movs	r3, #0
 8004c22:	8003      	strh	r3, [r0, #0]
 8004c24:	8043      	strh	r3, [r0, #2]
 8004c26:	8083      	strh	r3, [r0, #4]
 8004c28:	80c3      	strh	r3, [r0, #6]
 8004c2a:	8103      	strh	r3, [r0, #8]
 8004c2c:	8143      	strh	r3, [r0, #10]
 8004c2e:	8183      	strh	r3, [r0, #12]
 8004c30:	81c3      	strh	r3, [r0, #14]
 8004c32:	2307      	movs	r3, #7
 8004c34:	8203      	strh	r3, [r0, #16]
 8004c36:	4770      	bx	lr

08004c38 <I2S_StructInit>:
 8004c38:	2300      	movs	r3, #0
 8004c3a:	2202      	movs	r2, #2
 8004c3c:	8003      	strh	r3, [r0, #0]
 8004c3e:	8043      	strh	r3, [r0, #2]
 8004c40:	8083      	strh	r3, [r0, #4]
 8004c42:	80c3      	strh	r3, [r0, #6]
 8004c44:	8102      	strh	r2, [r0, #8]
 8004c46:	8143      	strh	r3, [r0, #10]
 8004c48:	4770      	bx	lr

08004c4a <SPI_Cmd>:
 8004c4a:	b121      	cbz	r1, 8004c56 <SPI_Cmd+0xc>
 8004c4c:	8803      	ldrh	r3, [r0, #0]
 8004c4e:	b29b      	uxth	r3, r3
 8004c50:	f043 0340 	orr.w	r3, r3, #64	; 0x40
 8004c54:	e004      	b.n	8004c60 <SPI_Cmd+0x16>
 8004c56:	8803      	ldrh	r3, [r0, #0]
 8004c58:	f023 0340 	bic.w	r3, r3, #64	; 0x40
 8004c5c:	041b      	lsls	r3, r3, #16
 8004c5e:	0c1b      	lsrs	r3, r3, #16
 8004c60:	8003      	strh	r3, [r0, #0]
 8004c62:	4770      	bx	lr

08004c64 <I2S_Cmd>:
 8004c64:	b121      	cbz	r1, 8004c70 <I2S_Cmd+0xc>
 8004c66:	8b83      	ldrh	r3, [r0, #28]
 8004c68:	b29b      	uxth	r3, r3
 8004c6a:	f443 6380 	orr.w	r3, r3, #1024	; 0x400
 8004c6e:	e004      	b.n	8004c7a <I2S_Cmd+0x16>
 8004c70:	8b83      	ldrh	r3, [r0, #28]
 8004c72:	f423 6380 	bic.w	r3, r3, #1024	; 0x400
 8004c76:	041b      	lsls	r3, r3, #16
 8004c78:	0c1b      	lsrs	r3, r3, #16
 8004c7a:	8383      	strh	r3, [r0, #28]
 8004c7c:	4770      	bx	lr

08004c7e <SPI_I2S_ITConfig>:
 8004c7e:	0909      	lsrs	r1, r1, #4
 8004c80:	2301      	movs	r3, #1
 8004c82:	408b      	lsls	r3, r1
 8004c84:	b29b      	uxth	r3, r3
 8004c86:	b11a      	cbz	r2, 8004c90 <SPI_I2S_ITConfig+0x12>
 8004c88:	8882      	ldrh	r2, [r0, #4]
 8004c8a:	b292      	uxth	r2, r2
 8004c8c:	4313      	orrs	r3, r2
 8004c8e:	e003      	b.n	8004c98 <SPI_I2S_ITConfig+0x1a>
 8004c90:	8882      	ldrh	r2, [r0, #4]
 8004c92:	b292      	uxth	r2, r2
 8004c94:	ea22 0303 	bic.w	r3, r2, r3
 8004c98:	8083      	strh	r3, [r0, #4]
 8004c9a:	4770      	bx	lr

08004c9c <SPI_I2S_DMACmd>:
 8004c9c:	b11a      	cbz	r2, 8004ca6 <SPI_I2S_DMACmd+0xa>
 8004c9e:	8883      	ldrh	r3, [r0, #4]
 8004ca0:	b29b      	uxth	r3, r3
 8004ca2:	4319      	orrs	r1, r3
 8004ca4:	e003      	b.n	8004cae <SPI_I2S_DMACmd+0x12>
 8004ca6:	8883      	ldrh	r3, [r0, #4]
 8004ca8:	b29b      	uxth	r3, r3
 8004caa:	ea23 0101 	bic.w	r1, r3, r1
 8004cae:	8081      	strh	r1, [r0, #4]
 8004cb0:	4770      	bx	lr

08004cb2 <SPI_I2S_SendData>:
 8004cb2:	8181      	strh	r1, [r0, #12]
 8004cb4:	4770      	bx	lr

08004cb6 <SPI_I2S_ReceiveData>:
 8004cb6:	8980      	ldrh	r0, [r0, #12]
 8004cb8:	b280      	uxth	r0, r0
 8004cba:	4770      	bx	lr

08004cbc <SPI_NSSInternalSoftwareConfig>:
 8004cbc:	f64f 63ff 	movw	r3, #65279	; 0xfeff
 8004cc0:	4299      	cmp	r1, r3
 8004cc2:	d004      	beq.n	8004cce <SPI_NSSInternalSoftwareConfig+0x12>
 8004cc4:	8803      	ldrh	r3, [r0, #0]
 8004cc6:	b29b      	uxth	r3, r3
 8004cc8:	f443 7380 	orr.w	r3, r3, #256	; 0x100
 8004ccc:	e004      	b.n	8004cd8 <SPI_NSSInternalSoftwareConfig+0x1c>
 8004cce:	8803      	ldrh	r3, [r0, #0]
 8004cd0:	f423 7380 	bic.w	r3, r3, #256	; 0x100
 8004cd4:	041b      	lsls	r3, r3, #16
 8004cd6:	0c1b      	lsrs	r3, r3, #16
 8004cd8:	8003      	strh	r3, [r0, #0]
 8004cda:	4770      	bx	lr

08004cdc <SPI_SSOutputCmd>:
 8004cdc:	b121      	cbz	r1, 8004ce8 <SPI_SSOutputCmd+0xc>
 8004cde:	8883      	ldrh	r3, [r0, #4]
 8004ce0:	b29b      	uxth	r3, r3
 8004ce2:	f043 0304 	orr.w	r3, r3, #4
 8004ce6:	e004      	b.n	8004cf2 <SPI_SSOutputCmd+0x16>
 8004ce8:	8883      	ldrh	r3, [r0, #4]
 8004cea:	f023 0304 	bic.w	r3, r3, #4
 8004cee:	041b      	lsls	r3, r3, #16
 8004cf0:	0c1b      	lsrs	r3, r3, #16
 8004cf2:	8083      	strh	r3, [r0, #4]
 8004cf4:	4770      	bx	lr

08004cf6 <SPI_DataSizeConfig>:
 8004cf6:	8803      	ldrh	r3, [r0, #0]
 8004cf8:	f423 6300 	bic.w	r3, r3, #2048	; 0x800
 8004cfc:	041b      	lsls	r3, r3, #16
 8004cfe:	0c1b      	lsrs	r3, r3, #16
 8004d00:	8003      	strh	r3, [r0, #0]
 8004d02:	8803      	ldrh	r3, [r0, #0]
 8004d04:	b29b      	uxth	r3, r3
 8004d06:	ea41 0303 	orr.w	r3, r1, r3
 8004d0a:	8003      	strh	r3, [r0, #0]
 8004d0c:	4770      	bx	lr

08004d0e <SPI_TransmitCRC>:
 8004d0e:	8803      	ldrh	r3, [r0, #0]
 8004d10:	b29b      	uxth	r3, r3
 8004d12:	f443 5380 	orr.w	r3, r3, #4096	; 0x1000
 8004d16:	8003      	strh	r3, [r0, #0]
 8004d18:	4770      	bx	lr

08004d1a <SPI_CalculateCRC>:
 8004d1a:	b121      	cbz	r1, 8004d26 <SPI_CalculateCRC+0xc>
 8004d1c:	8803      	ldrh	r3, [r0, #0]
 8004d1e:	b29b      	uxth	r3, r3
 8004d20:	f443 5300 	orr.w	r3, r3, #8192	; 0x2000
 8004d24:	e004      	b.n	8004d30 <SPI_CalculateCRC+0x16>
 8004d26:	8803      	ldrh	r3, [r0, #0]
 8004d28:	f423 5300 	bic.w	r3, r3, #8192	; 0x2000
 8004d2c:	041b      	lsls	r3, r3, #16
 8004d2e:	0c1b      	lsrs	r3, r3, #16
 8004d30:	8003      	strh	r3, [r0, #0]
 8004d32:	4770      	bx	lr

08004d34 <SPI_GetCRC>:
 8004d34:	2901      	cmp	r1, #1
 8004d36:	d002      	beq.n	8004d3e <SPI_GetCRC+0xa>
 8004d38:	8b00      	ldrh	r0, [r0, #24]
 8004d3a:	b280      	uxth	r0, r0
 8004d3c:	4770      	bx	lr
 8004d3e:	8a80      	ldrh	r0, [r0, #20]
 8004d40:	b280      	uxth	r0, r0
 8004d42:	4770      	bx	lr

08004d44 <SPI_GetCRCPolynomial>:
 8004d44:	8a00      	ldrh	r0, [r0, #16]
 8004d46:	b280      	uxth	r0, r0
 8004d48:	4770      	bx	lr

08004d4a <SPI_BiDirectionalLineConfig>:
 8004d4a:	f5b1 4f80 	cmp.w	r1, #16384	; 0x4000
 8004d4e:	d104      	bne.n	8004d5a <SPI_BiDirectionalLineConfig+0x10>
 8004d50:	8803      	ldrh	r3, [r0, #0]
 8004d52:	b29b      	uxth	r3, r3
 8004d54:	f443 4380 	orr.w	r3, r3, #16384	; 0x4000
 8004d58:	e004      	b.n	8004d64 <SPI_BiDirectionalLineConfig+0x1a>
 8004d5a:	8803      	ldrh	r3, [r0, #0]
 8004d5c:	f423 4380 	bic.w	r3, r3, #16384	; 0x4000
 8004d60:	041b      	lsls	r3, r3, #16
 8004d62:	0c1b      	lsrs	r3, r3, #16
 8004d64:	8003      	strh	r3, [r0, #0]
 8004d66:	4770      	bx	lr

08004d68 <SPI_I2S_GetFlagStatus>:
 8004d68:	8903      	ldrh	r3, [r0, #8]
 8004d6a:	4219      	tst	r1, r3
 8004d6c:	bf0c      	ite	eq
 8004d6e:	2000      	moveq	r0, #0
 8004d70:	2001      	movne	r0, #1
 8004d72:	4770      	bx	lr

08004d74 <SPI_I2S_ClearFlag>:
 8004d74:	43c9      	mvns	r1, r1
 8004d76:	8101      	strh	r1, [r0, #8]
 8004d78:	4770      	bx	lr

08004d7a <SPI_I2S_GetITStatus>:
 8004d7a:	2301      	movs	r3, #1
 8004d7c:	f001 020f 	and.w	r2, r1, #15
 8004d80:	fa13 f202 	lsls.w	r2, r3, r2
 8004d84:	b510      	push	{r4, lr}
 8004d86:	8884      	ldrh	r4, [r0, #4]
 8004d88:	8900      	ldrh	r0, [r0, #8]
 8004d8a:	b2a4      	uxth	r4, r4
 8004d8c:	b280      	uxth	r0, r0
 8004d8e:	4010      	ands	r0, r2
 8004d90:	d005      	beq.n	8004d9e <SPI_I2S_GetITStatus+0x24>
 8004d92:	0909      	lsrs	r1, r1, #4
 8004d94:	408b      	lsls	r3, r1
 8004d96:	421c      	tst	r4, r3
 8004d98:	bf0c      	ite	eq
 8004d9a:	2000      	moveq	r0, #0
 8004d9c:	2001      	movne	r0, #1
 8004d9e:	bd10      	pop	{r4, pc}

08004da0 <SPI_I2S_ClearITPendingBit>:
 8004da0:	f001 010f 	and.w	r1, r1, #15
 8004da4:	2301      	movs	r3, #1
 8004da6:	408b      	lsls	r3, r1
 8004da8:	43db      	mvns	r3, r3
 8004daa:	8103      	strh	r3, [r0, #8]
 8004dac:	4770      	bx	lr
 8004dae:	bf00      	nop

08004db0 <USART_DeInit>:
 8004db0:	4b20      	ldr	r3, [pc, #128]	; (8004e34 <USART_DeInit+0x84>)
 8004db2:	b510      	push	{r4, lr}
 8004db4:	4298      	cmp	r0, r3
 8004db6:	d10b      	bne.n	8004dd0 <USART_DeInit+0x20>
 8004db8:	2101      	movs	r1, #1
 8004dba:	f44f 4080 	mov.w	r0, #16384	; 0x4000
 8004dbe:	f7ff fe21 	bl	8004a04 <RCC_APB2PeriphResetCmd>
 8004dc2:	f44f 4080 	mov.w	r0, #16384	; 0x4000
 8004dc6:	2100      	movs	r1, #0
 8004dc8:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 8004dcc:	f7ff be1a 	b.w	8004a04 <RCC_APB2PeriphResetCmd>
 8004dd0:	4b19      	ldr	r3, [pc, #100]	; (8004e38 <USART_DeInit+0x88>)
 8004dd2:	4298      	cmp	r0, r3
 8004dd4:	d107      	bne.n	8004de6 <USART_DeInit+0x36>
 8004dd6:	f44f 3000 	mov.w	r0, #131072	; 0x20000
 8004dda:	2101      	movs	r1, #1
 8004ddc:	f7ff fe1e 	bl	8004a1c <RCC_APB1PeriphResetCmd>
 8004de0:	f44f 3000 	mov.w	r0, #131072	; 0x20000
 8004de4:	e01f      	b.n	8004e26 <USART_DeInit+0x76>
 8004de6:	4b15      	ldr	r3, [pc, #84]	; (8004e3c <USART_DeInit+0x8c>)
 8004de8:	4298      	cmp	r0, r3
 8004dea:	d107      	bne.n	8004dfc <USART_DeInit+0x4c>
 8004dec:	f44f 2080 	mov.w	r0, #262144	; 0x40000
 8004df0:	2101      	movs	r1, #1
 8004df2:	f7ff fe13 	bl	8004a1c <RCC_APB1PeriphResetCmd>
 8004df6:	f44f 2080 	mov.w	r0, #262144	; 0x40000
 8004dfa:	e014      	b.n	8004e26 <USART_DeInit+0x76>
 8004dfc:	4b10      	ldr	r3, [pc, #64]	; (8004e40 <USART_DeInit+0x90>)
 8004dfe:	4298      	cmp	r0, r3
 8004e00:	d107      	bne.n	8004e12 <USART_DeInit+0x62>
 8004e02:	f44f 2000 	mov.w	r0, #524288	; 0x80000
 8004e06:	2101      	movs	r1, #1
 8004e08:	f7ff fe08 	bl	8004a1c <RCC_APB1PeriphResetCmd>
 8004e0c:	f44f 2000 	mov.w	r0, #524288	; 0x80000
 8004e10:	e009      	b.n	8004e26 <USART_DeInit+0x76>
 8004e12:	4b0c      	ldr	r3, [pc, #48]	; (8004e44 <USART_DeInit+0x94>)
 8004e14:	4298      	cmp	r0, r3
 8004e16:	d10b      	bne.n	8004e30 <USART_DeInit+0x80>
 8004e18:	f44f 1080 	mov.w	r0, #1048576	; 0x100000
 8004e1c:	2101      	movs	r1, #1
 8004e1e:	f7ff fdfd 	bl	8004a1c <RCC_APB1PeriphResetCmd>
 8004e22:	f44f 1080 	mov.w	r0, #1048576	; 0x100000
 8004e26:	2100      	movs	r1, #0
 8004e28:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 8004e2c:	f7ff bdf6 	b.w	8004a1c <RCC_APB1PeriphResetCmd>
 8004e30:	bd10      	pop	{r4, pc}
 8004e32:	bf00      	nop
 8004e34:	40013800 	.word	0x40013800
 8004e38:	40004400 	.word	0x40004400
 8004e3c:	40004800 	.word	0x40004800
 8004e40:	40004c00 	.word	0x40004c00
 8004e44:	40005000 	.word	0x40005000

08004e48 <USART_Init>:
 8004e48:	b530      	push	{r4, r5, lr}
 8004e4a:	8a03      	ldrh	r3, [r0, #16]
 8004e4c:	88ca      	ldrh	r2, [r1, #6]
 8004e4e:	f423 5340 	bic.w	r3, r3, #12288	; 0x3000
 8004e52:	041b      	lsls	r3, r3, #16
 8004e54:	0c1b      	lsrs	r3, r3, #16
 8004e56:	4313      	orrs	r3, r2
 8004e58:	8203      	strh	r3, [r0, #16]
 8004e5a:	460d      	mov	r5, r1
 8004e5c:	8983      	ldrh	r3, [r0, #12]
 8004e5e:	8909      	ldrh	r1, [r1, #8]
 8004e60:	88aa      	ldrh	r2, [r5, #4]
 8004e62:	f423 53b0 	bic.w	r3, r3, #5632	; 0x1600
 8004e66:	ea41 0202 	orr.w	r2, r1, r2
 8004e6a:	8969      	ldrh	r1, [r5, #10]
 8004e6c:	f023 030c 	bic.w	r3, r3, #12
 8004e70:	430a      	orrs	r2, r1
 8004e72:	041b      	lsls	r3, r3, #16
 8004e74:	0c1b      	lsrs	r3, r3, #16
 8004e76:	b292      	uxth	r2, r2
 8004e78:	ea42 0303 	orr.w	r3, r2, r3
 8004e7c:	8183      	strh	r3, [r0, #12]
 8004e7e:	8a83      	ldrh	r3, [r0, #20]
 8004e80:	89aa      	ldrh	r2, [r5, #12]
 8004e82:	f423 7340 	bic.w	r3, r3, #768	; 0x300
 8004e86:	041b      	lsls	r3, r3, #16
 8004e88:	0c1b      	lsrs	r3, r3, #16
 8004e8a:	b087      	sub	sp, #28
 8004e8c:	4313      	orrs	r3, r2
 8004e8e:	4604      	mov	r4, r0
 8004e90:	8283      	strh	r3, [r0, #20]
 8004e92:	a801      	add	r0, sp, #4
 8004e94:	f7ff fd40 	bl	8004918 <RCC_GetClocksFreq>
 8004e98:	4b0e      	ldr	r3, [pc, #56]	; (8004ed4 <USART_Init+0x8c>)
 8004e9a:	682a      	ldr	r2, [r5, #0]
 8004e9c:	429c      	cmp	r4, r3
 8004e9e:	bf0c      	ite	eq
 8004ea0:	9b04      	ldreq	r3, [sp, #16]
 8004ea2:	9b03      	ldrne	r3, [sp, #12]
 8004ea4:	2119      	movs	r1, #25
 8004ea6:	434b      	muls	r3, r1
 8004ea8:	0092      	lsls	r2, r2, #2
 8004eaa:	fbb3 f1f2 	udiv	r1, r3, r2
 8004eae:	2364      	movs	r3, #100	; 0x64
 8004eb0:	fbb1 f2f3 	udiv	r2, r1, r3
 8004eb4:	0112      	lsls	r2, r2, #4
 8004eb6:	0910      	lsrs	r0, r2, #4
 8004eb8:	fb03 1110 	mls	r1, r3, r0, r1
 8004ebc:	0109      	lsls	r1, r1, #4
 8004ebe:	3132      	adds	r1, #50	; 0x32
 8004ec0:	fbb1 f3f3 	udiv	r3, r1, r3
 8004ec4:	f003 030f 	and.w	r3, r3, #15
 8004ec8:	ea43 0202 	orr.w	r2, r3, r2
 8004ecc:	8122      	strh	r2, [r4, #8]
 8004ece:	b007      	add	sp, #28
 8004ed0:	bd30      	pop	{r4, r5, pc}
 8004ed2:	bf00      	nop
 8004ed4:	40013800 	.word	0x40013800

08004ed8 <USART_StructInit>:
 8004ed8:	f44f 5316 	mov.w	r3, #9600	; 0x2580
 8004edc:	6003      	str	r3, [r0, #0]
 8004ede:	220c      	movs	r2, #12
 8004ee0:	2300      	movs	r3, #0
 8004ee2:	8083      	strh	r3, [r0, #4]
 8004ee4:	80c3      	strh	r3, [r0, #6]
 8004ee6:	8103      	strh	r3, [r0, #8]
 8004ee8:	8142      	strh	r2, [r0, #10]
 8004eea:	8183      	strh	r3, [r0, #12]
 8004eec:	4770      	bx	lr

08004eee <USART_ClockInit>:
 8004eee:	b510      	push	{r4, lr}
 8004ef0:	884c      	ldrh	r4, [r1, #2]
 8004ef2:	880a      	ldrh	r2, [r1, #0]
 8004ef4:	8a03      	ldrh	r3, [r0, #16]
 8004ef6:	ea44 0202 	orr.w	r2, r4, r2
 8004efa:	888c      	ldrh	r4, [r1, #4]
 8004efc:	88c9      	ldrh	r1, [r1, #6]
 8004efe:	f423 6370 	bic.w	r3, r3, #3840	; 0xf00
 8004f02:	4322      	orrs	r2, r4
 8004f04:	041b      	lsls	r3, r3, #16
 8004f06:	430a      	orrs	r2, r1
 8004f08:	0c1b      	lsrs	r3, r3, #16
 8004f0a:	b292      	uxth	r2, r2
 8004f0c:	ea42 0303 	orr.w	r3, r2, r3
 8004f10:	8203      	strh	r3, [r0, #16]
 8004f12:	bd10      	pop	{r4, pc}

08004f14 <USART_ClockStructInit>:
 8004f14:	2300      	movs	r3, #0
 8004f16:	8003      	strh	r3, [r0, #0]
 8004f18:	8043      	strh	r3, [r0, #2]
 8004f1a:	8083      	strh	r3, [r0, #4]
 8004f1c:	80c3      	strh	r3, [r0, #6]
 8004f1e:	4770      	bx	lr

08004f20 <USART_Cmd>:
 8004f20:	b121      	cbz	r1, 8004f2c <USART_Cmd+0xc>
 8004f22:	8983      	ldrh	r3, [r0, #12]
 8004f24:	b29b      	uxth	r3, r3
 8004f26:	f443 5300 	orr.w	r3, r3, #8192	; 0x2000
 8004f2a:	e004      	b.n	8004f36 <USART_Cmd+0x16>
 8004f2c:	8983      	ldrh	r3, [r0, #12]
 8004f2e:	f423 5300 	bic.w	r3, r3, #8192	; 0x2000
 8004f32:	041b      	lsls	r3, r3, #16
 8004f34:	0c1b      	lsrs	r3, r3, #16
 8004f36:	8183      	strh	r3, [r0, #12]
 8004f38:	4770      	bx	lr

08004f3a <USART_ITConfig>:
 8004f3a:	b2cb      	uxtb	r3, r1
 8004f3c:	b510      	push	{r4, lr}
 8004f3e:	095b      	lsrs	r3, r3, #5
 8004f40:	2401      	movs	r4, #1
 8004f42:	f001 011f 	and.w	r1, r1, #31
 8004f46:	fa14 f101 	lsls.w	r1, r4, r1
 8004f4a:	42a3      	cmp	r3, r4
 8004f4c:	d101      	bne.n	8004f52 <USART_ITConfig+0x18>
 8004f4e:	300c      	adds	r0, #12
 8004f50:	e004      	b.n	8004f5c <USART_ITConfig+0x22>
 8004f52:	2b02      	cmp	r3, #2
 8004f54:	d101      	bne.n	8004f5a <USART_ITConfig+0x20>
 8004f56:	3010      	adds	r0, #16
 8004f58:	e000      	b.n	8004f5c <USART_ITConfig+0x22>
 8004f5a:	3014      	adds	r0, #20
 8004f5c:	b11a      	cbz	r2, 8004f66 <USART_ITConfig+0x2c>
 8004f5e:	6803      	ldr	r3, [r0, #0]
 8004f60:	ea43 0101 	orr.w	r1, r3, r1
 8004f64:	e002      	b.n	8004f6c <USART_ITConfig+0x32>
 8004f66:	6803      	ldr	r3, [r0, #0]
 8004f68:	ea23 0101 	bic.w	r1, r3, r1
 8004f6c:	6001      	str	r1, [r0, #0]
 8004f6e:	bd10      	pop	{r4, pc}

08004f70 <USART_DMACmd>:
 8004f70:	b11a      	cbz	r2, 8004f7a <USART_DMACmd+0xa>
 8004f72:	8a83      	ldrh	r3, [r0, #20]
 8004f74:	b29b      	uxth	r3, r3
 8004f76:	4319      	orrs	r1, r3
 8004f78:	e003      	b.n	8004f82 <USART_DMACmd+0x12>
 8004f7a:	8a83      	ldrh	r3, [r0, #20]
 8004f7c:	b29b      	uxth	r3, r3
 8004f7e:	ea23 0101 	bic.w	r1, r3, r1
 8004f82:	8281      	strh	r1, [r0, #20]
 8004f84:	4770      	bx	lr

08004f86 <USART_SetAddress>:
 8004f86:	8a03      	ldrh	r3, [r0, #16]
 8004f88:	f023 030f 	bic.w	r3, r3, #15
 8004f8c:	041b      	lsls	r3, r3, #16
 8004f8e:	0c1b      	lsrs	r3, r3, #16
 8004f90:	8203      	strh	r3, [r0, #16]
 8004f92:	8a03      	ldrh	r3, [r0, #16]
 8004f94:	b29b      	uxth	r3, r3
 8004f96:	ea41 0303 	orr.w	r3, r1, r3
 8004f9a:	8203      	strh	r3, [r0, #16]
 8004f9c:	4770      	bx	lr

08004f9e <USART_WakeUpConfig>:
 8004f9e:	8983      	ldrh	r3, [r0, #12]
 8004fa0:	f423 6300 	bic.w	r3, r3, #2048	; 0x800
 8004fa4:	041b      	lsls	r3, r3, #16
 8004fa6:	0c1b      	lsrs	r3, r3, #16
 8004fa8:	8183      	strh	r3, [r0, #12]
 8004faa:	8983      	ldrh	r3, [r0, #12]
 8004fac:	b29b      	uxth	r3, r3
 8004fae:	ea41 0303 	orr.w	r3, r1, r3
 8004fb2:	8183      	strh	r3, [r0, #12]
 8004fb4:	4770      	bx	lr

08004fb6 <USART_ReceiverWakeUpCmd>:
 8004fb6:	b121      	cbz	r1, 8004fc2 <USART_ReceiverWakeUpCmd+0xc>
 8004fb8:	8983      	ldrh	r3, [r0, #12]
 8004fba:	b29b      	uxth	r3, r3
 8004fbc:	f043 0302 	orr.w	r3, r3, #2
 8004fc0:	e004      	b.n	8004fcc <USART_ReceiverWakeUpCmd+0x16>
 8004fc2:	8983      	ldrh	r3, [r0, #12]
 8004fc4:	f023 0302 	bic.w	r3, r3, #2
 8004fc8:	041b      	lsls	r3, r3, #16
 8004fca:	0c1b      	lsrs	r3, r3, #16
 8004fcc:	8183      	strh	r3, [r0, #12]
 8004fce:	4770      	bx	lr

08004fd0 <USART_LINBreakDetectLengthConfig>:
 8004fd0:	8a03      	ldrh	r3, [r0, #16]
 8004fd2:	f023 0320 	bic.w	r3, r3, #32
 8004fd6:	041b      	lsls	r3, r3, #16
 8004fd8:	0c1b      	lsrs	r3, r3, #16
 8004fda:	8203      	strh	r3, [r0, #16]
 8004fdc:	8a03      	ldrh	r3, [r0, #16]
 8004fde:	b29b      	uxth	r3, r3
 8004fe0:	ea41 0303 	orr.w	r3, r1, r3
 8004fe4:	8203      	strh	r3, [r0, #16]
 8004fe6:	4770      	bx	lr

08004fe8 <USART_LINCmd>:
 8004fe8:	b121      	cbz	r1, 8004ff4 <USART_LINCmd+0xc>
 8004fea:	8a03      	ldrh	r3, [r0, #16]
 8004fec:	b29b      	uxth	r3, r3
 8004fee:	f443 4380 	orr.w	r3, r3, #16384	; 0x4000
 8004ff2:	e004      	b.n	8004ffe <USART_LINCmd+0x16>
 8004ff4:	8a03      	ldrh	r3, [r0, #16]
 8004ff6:	f423 4380 	bic.w	r3, r3, #16384	; 0x4000
 8004ffa:	041b      	lsls	r3, r3, #16
 8004ffc:	0c1b      	lsrs	r3, r3, #16
 8004ffe:	8203      	strh	r3, [r0, #16]
 8005000:	4770      	bx	lr

08005002 <USART_SendData>:
 8005002:	05c9      	lsls	r1, r1, #23
 8005004:	0dc9      	lsrs	r1, r1, #23
 8005006:	8081      	strh	r1, [r0, #4]
 8005008:	4770      	bx	lr

0800500a <USART_ReceiveData>:
 800500a:	8880      	ldrh	r0, [r0, #4]
 800500c:	05c0      	lsls	r0, r0, #23
 800500e:	0dc0      	lsrs	r0, r0, #23
 8005010:	4770      	bx	lr

08005012 <USART_SendBreak>:
 8005012:	8983      	ldrh	r3, [r0, #12]
 8005014:	b29b      	uxth	r3, r3
 8005016:	f043 0301 	orr.w	r3, r3, #1
 800501a:	8183      	strh	r3, [r0, #12]
 800501c:	4770      	bx	lr

0800501e <USART_SetGuardTime>:
 800501e:	8b03      	ldrh	r3, [r0, #24]
 8005020:	b2db      	uxtb	r3, r3
 8005022:	8303      	strh	r3, [r0, #24]
 8005024:	8b03      	ldrh	r3, [r0, #24]
 8005026:	b29b      	uxth	r3, r3
 8005028:	ea43 2301 	orr.w	r3, r3, r1, lsl #8
 800502c:	8303      	strh	r3, [r0, #24]
 800502e:	4770      	bx	lr

08005030 <USART_SetPrescaler>:
 8005030:	8b03      	ldrh	r3, [r0, #24]
 8005032:	f403 437f 	and.w	r3, r3, #65280	; 0xff00
 8005036:	8303      	strh	r3, [r0, #24]
 8005038:	8b03      	ldrh	r3, [r0, #24]
 800503a:	b29b      	uxth	r3, r3
 800503c:	ea41 0303 	orr.w	r3, r1, r3
 8005040:	8303      	strh	r3, [r0, #24]
 8005042:	4770      	bx	lr

08005044 <USART_SmartCardCmd>:
 8005044:	b121      	cbz	r1, 8005050 <USART_SmartCardCmd+0xc>
 8005046:	8a83      	ldrh	r3, [r0, #20]
 8005048:	b29b      	uxth	r3, r3
 800504a:	f043 0320 	orr.w	r3, r3, #32
 800504e:	e004      	b.n	800505a <USART_SmartCardCmd+0x16>
 8005050:	8a83      	ldrh	r3, [r0, #20]
 8005052:	f023 0320 	bic.w	r3, r3, #32
 8005056:	041b      	lsls	r3, r3, #16
 8005058:	0c1b      	lsrs	r3, r3, #16
 800505a:	8283      	strh	r3, [r0, #20]
 800505c:	4770      	bx	lr

0800505e <USART_SmartCardNACKCmd>:
 800505e:	b121      	cbz	r1, 800506a <USART_SmartCardNACKCmd+0xc>
 8005060:	8a83      	ldrh	r3, [r0, #20]
 8005062:	b29b      	uxth	r3, r3
 8005064:	f043 0310 	orr.w	r3, r3, #16
 8005068:	e004      	b.n	8005074 <USART_SmartCardNACKCmd+0x16>
 800506a:	8a83      	ldrh	r3, [r0, #20]
 800506c:	f023 0310 	bic.w	r3, r3, #16
 8005070:	041b      	lsls	r3, r3, #16
 8005072:	0c1b      	lsrs	r3, r3, #16
 8005074:	8283      	strh	r3, [r0, #20]
 8005076:	4770      	bx	lr

08005078 <USART_HalfDuplexCmd>:
 8005078:	b121      	cbz	r1, 8005084 <USART_HalfDuplexCmd+0xc>
 800507a:	8a83      	ldrh	r3, [r0, #20]
 800507c:	b29b      	uxth	r3, r3
 800507e:	f043 0308 	orr.w	r3, r3, #8
 8005082:	e004      	b.n	800508e <USART_HalfDuplexCmd+0x16>
 8005084:	8a83      	ldrh	r3, [r0, #20]
 8005086:	f023 0308 	bic.w	r3, r3, #8
 800508a:	041b      	lsls	r3, r3, #16
 800508c:	0c1b      	lsrs	r3, r3, #16
 800508e:	8283      	strh	r3, [r0, #20]
 8005090:	4770      	bx	lr

08005092 <USART_IrDAConfig>:
 8005092:	8a83      	ldrh	r3, [r0, #20]
 8005094:	f023 0304 	bic.w	r3, r3, #4
 8005098:	041b      	lsls	r3, r3, #16
 800509a:	0c1b      	lsrs	r3, r3, #16
 800509c:	8283      	strh	r3, [r0, #20]
 800509e:	8a83      	ldrh	r3, [r0, #20]
 80050a0:	b29b      	uxth	r3, r3
 80050a2:	ea41 0303 	orr.w	r3, r1, r3
 80050a6:	8283      	strh	r3, [r0, #20]
 80050a8:	4770      	bx	lr

080050aa <USART_IrDACmd>:
 80050aa:	b121      	cbz	r1, 80050b6 <USART_IrDACmd+0xc>
 80050ac:	8a83      	ldrh	r3, [r0, #20]
 80050ae:	b29b      	uxth	r3, r3
 80050b0:	f043 0302 	orr.w	r3, r3, #2
 80050b4:	e004      	b.n	80050c0 <USART_IrDACmd+0x16>
 80050b6:	8a83      	ldrh	r3, [r0, #20]
 80050b8:	f023 0302 	bic.w	r3, r3, #2
 80050bc:	041b      	lsls	r3, r3, #16
 80050be:	0c1b      	lsrs	r3, r3, #16
 80050c0:	8283      	strh	r3, [r0, #20]
 80050c2:	4770      	bx	lr

080050c4 <USART_GetFlagStatus>:
 80050c4:	8803      	ldrh	r3, [r0, #0]
 80050c6:	4219      	tst	r1, r3
 80050c8:	bf0c      	ite	eq
 80050ca:	2000      	moveq	r0, #0
 80050cc:	2001      	movne	r0, #1
 80050ce:	4770      	bx	lr

080050d0 <USART_ClearFlag>:
 80050d0:	43c9      	mvns	r1, r1
 80050d2:	8001      	strh	r1, [r0, #0]
 80050d4:	4770      	bx	lr

080050d6 <USART_GetITStatus>:
 80050d6:	b510      	push	{r4, lr}
 80050d8:	b2cc      	uxtb	r4, r1
 80050da:	2201      	movs	r2, #1
 80050dc:	0964      	lsrs	r4, r4, #5
 80050de:	f001 031f 	and.w	r3, r1, #31
 80050e2:	409a      	lsls	r2, r3
 80050e4:	2c01      	cmp	r4, #1
 80050e6:	d101      	bne.n	80050ec <USART_GetITStatus+0x16>
 80050e8:	8983      	ldrh	r3, [r0, #12]
 80050ea:	e003      	b.n	80050f4 <USART_GetITStatus+0x1e>
 80050ec:	2c02      	cmp	r4, #2
 80050ee:	bf0c      	ite	eq
 80050f0:	8a03      	ldrheq	r3, [r0, #16]
 80050f2:	8a83      	ldrhne	r3, [r0, #20]
 80050f4:	b29b      	uxth	r3, r3
 80050f6:	ea02 0303 	and.w	r3, r2, r3
 80050fa:	8802      	ldrh	r2, [r0, #0]
 80050fc:	b292      	uxth	r2, r2
 80050fe:	b143      	cbz	r3, 8005112 <USART_GetITStatus+0x3c>
 8005100:	0a09      	lsrs	r1, r1, #8
 8005102:	2301      	movs	r3, #1
 8005104:	fa13 f101 	lsls.w	r1, r3, r1
 8005108:	4211      	tst	r1, r2
 800510a:	bf0c      	ite	eq
 800510c:	2000      	moveq	r0, #0
 800510e:	2001      	movne	r0, #1
 8005110:	bd10      	pop	{r4, pc}
 8005112:	4618      	mov	r0, r3
 8005114:	bd10      	pop	{r4, pc}

08005116 <USART_ClearITPendingBit>:
 8005116:	0a09      	lsrs	r1, r1, #8
 8005118:	2301      	movs	r3, #1
 800511a:	408b      	lsls	r3, r1
 800511c:	43db      	mvns	r3, r3
 800511e:	8003      	strh	r3, [r0, #0]
 8005120:	4770      	bx	lr
 8005122:	bf00      	nop

08005124 <Standard_GetConfiguration>:
 8005124:	b508      	push	{r3, lr}
 8005126:	b138      	cbz	r0, 8005138 <Standard_GetConfiguration+0x14>
 8005128:	4906      	ldr	r1, [pc, #24]	; (8005144 <Standard_GetConfiguration+0x20>)
 800512a:	680b      	ldr	r3, [r1, #0]
 800512c:	681a      	ldr	r2, [r3, #0]
 800512e:	4790      	blx	r2
 8005130:	4805      	ldr	r0, [pc, #20]	; (8005148 <Standard_GetConfiguration+0x24>)
 8005132:	6800      	ldr	r0, [r0, #0]
 8005134:	300a      	adds	r0, #10
 8005136:	bd08      	pop	{r3, pc}
 8005138:	4903      	ldr	r1, [pc, #12]	; (8005148 <Standard_GetConfiguration+0x24>)
 800513a:	2201      	movs	r2, #1
 800513c:	680b      	ldr	r3, [r1, #0]
 800513e:	821a      	strh	r2, [r3, #16]
 8005140:	bd08      	pop	{r3, pc}
 8005142:	bf00      	nop
 8005144:	20000508 	.word	0x20000508
 8005148:	2000050c 	.word	0x2000050c

0800514c <Standard_GetInterface>:
 800514c:	b508      	push	{r3, lr}
 800514e:	b138      	cbz	r0, 8005160 <Standard_GetInterface+0x14>
 8005150:	4906      	ldr	r1, [pc, #24]	; (800516c <Standard_GetInterface+0x20>)
 8005152:	680b      	ldr	r3, [r1, #0]
 8005154:	689a      	ldr	r2, [r3, #8]
 8005156:	4790      	blx	r2
 8005158:	4805      	ldr	r0, [pc, #20]	; (8005170 <Standard_GetInterface+0x24>)
 800515a:	6800      	ldr	r0, [r0, #0]
 800515c:	300c      	adds	r0, #12
 800515e:	bd08      	pop	{r3, pc}
 8005160:	4903      	ldr	r1, [pc, #12]	; (8005170 <Standard_GetInterface+0x24>)
 8005162:	2201      	movs	r2, #1
 8005164:	680b      	ldr	r3, [r1, #0]
 8005166:	821a      	strh	r2, [r3, #16]
 8005168:	bd08      	pop	{r3, pc}
 800516a:	bf00      	nop
 800516c:	20000508 	.word	0x20000508
 8005170:	2000050c 	.word	0x2000050c

08005174 <Standard_GetStatus>:
 8005174:	b508      	push	{r3, lr}
 8005176:	2800      	cmp	r0, #0
 8005178:	d029      	beq.n	80051ce <Standard_GetStatus+0x5a>
 800517a:	4a2a      	ldr	r2, [pc, #168]	; (8005224 <Standard_GetStatus+0xb0>)
 800517c:	482a      	ldr	r0, [pc, #168]	; (8005228 <Standard_GetStatus+0xb4>)
 800517e:	2300      	movs	r3, #0
 8005180:	6812      	ldr	r2, [r2, #0]
 8005182:	8003      	strh	r3, [r0, #0]
 8005184:	f892 c000 	ldrb.w	ip, [r2]
 8005188:	f01c 037f 	ands.w	r3, ip, #127	; 0x7f
 800518c:	d118      	bne.n	80051c0 <Standard_GetStatus+0x4c>
 800518e:	7a51      	ldrb	r1, [r2, #9]
 8005190:	f890 e000 	ldrb.w	lr, [r0]
 8005194:	f011 0f20 	tst.w	r1, #32
 8005198:	bf14      	ite	ne
 800519a:	f04e 0e02 	orrne.w	lr, lr, #2
 800519e:	f00e 0efd 	andeq.w	lr, lr, #253	; 0xfd
 80051a2:	f011 0f40 	tst.w	r1, #64	; 0x40
 80051a6:	bf14      	ite	ne
 80051a8:	f04e 0e01 	orrne.w	lr, lr, #1
 80051ac:	f02e 0e01 	biceq.w	lr, lr, #1
 80051b0:	f880 e000 	strb.w	lr, [r0]
 80051b4:	4b1d      	ldr	r3, [pc, #116]	; (800522c <Standard_GetStatus+0xb8>)
 80051b6:	6819      	ldr	r1, [r3, #0]
 80051b8:	6908      	ldr	r0, [r1, #16]
 80051ba:	4780      	blx	r0
 80051bc:	481a      	ldr	r0, [pc, #104]	; (8005228 <Standard_GetStatus+0xb4>)
 80051be:	bd08      	pop	{r3, pc}
 80051c0:	2b01      	cmp	r3, #1
 80051c2:	d100      	bne.n	80051c6 <Standard_GetStatus+0x52>
 80051c4:	bd08      	pop	{r3, pc}
 80051c6:	2b02      	cmp	r3, #2
 80051c8:	d006      	beq.n	80051d8 <Standard_GetStatus+0x64>
 80051ca:	2000      	movs	r0, #0
 80051cc:	e7fa      	b.n	80051c4 <Standard_GetStatus+0x50>
 80051ce:	4915      	ldr	r1, [pc, #84]	; (8005224 <Standard_GetStatus+0xb0>)
 80051d0:	2202      	movs	r2, #2
 80051d2:	680b      	ldr	r3, [r1, #0]
 80051d4:	821a      	strh	r2, [r3, #16]
 80051d6:	bd08      	pop	{r3, pc}
 80051d8:	f892 c005 	ldrb.w	ip, [r2, #5]
 80051dc:	f01c 0f80 	tst.w	ip, #128	; 0x80
 80051e0:	f00c 020f 	and.w	r2, ip, #15
 80051e4:	d111      	bne.n	800520a <Standard_GetStatus+0x96>
 80051e6:	0093      	lsls	r3, r2, #2
 80051e8:	f103 4180 	add.w	r1, r3, #1073741824	; 0x40000000
 80051ec:	f501 4eb8 	add.w	lr, r1, #23552	; 0x5c00
 80051f0:	f8de 2000 	ldr.w	r2, [lr]
 80051f4:	f402 5c40 	and.w	ip, r2, #12288	; 0x3000
 80051f8:	f5bc 5f80 	cmp.w	ip, #4096	; 0x1000
 80051fc:	d1da      	bne.n	80051b4 <Standard_GetStatus+0x40>
 80051fe:	7802      	ldrb	r2, [r0, #0]
 8005200:	f042 0c01 	orr.w	ip, r2, #1
 8005204:	f880 c000 	strb.w	ip, [r0]
 8005208:	e7d4      	b.n	80051b4 <Standard_GetStatus+0x40>
 800520a:	0093      	lsls	r3, r2, #2
 800520c:	f103 4180 	add.w	r1, r3, #1073741824	; 0x40000000
 8005210:	f501 4eb8 	add.w	lr, r1, #23552	; 0x5c00
 8005214:	f8de 2000 	ldr.w	r2, [lr]
 8005218:	f002 0330 	and.w	r3, r2, #48	; 0x30
 800521c:	2b10      	cmp	r3, #16
 800521e:	d1c9      	bne.n	80051b4 <Standard_GetStatus+0x40>
 8005220:	e7ed      	b.n	80051fe <Standard_GetStatus+0x8a>
 8005222:	bf00      	nop
 8005224:	2000050c 	.word	0x2000050c
 8005228:	200004dc 	.word	0x200004dc
 800522c:	20000508 	.word	0x20000508

08005230 <DataStageIn>:
 8005230:	e92d 41f0 	stmdb	sp!, {r4, r5, r6, r7, r8, lr}
 8005234:	4e26      	ldr	r6, [pc, #152]	; (80052d0 <DataStageIn+0xa0>)
 8005236:	6834      	ldr	r4, [r6, #0]
 8005238:	8a23      	ldrh	r3, [r4, #16]
 800523a:	7a27      	ldrb	r7, [r4, #8]
 800523c:	b923      	cbnz	r3, 8005248 <DataStageIn+0x18>
 800523e:	2f04      	cmp	r7, #4
 8005240:	d02c      	beq.n	800529c <DataStageIn+0x6c>
 8005242:	8aa5      	ldrh	r5, [r4, #20]
 8005244:	2704      	movs	r7, #4
 8005246:	e003      	b.n	8005250 <DataStageIn+0x20>
 8005248:	8aa5      	ldrh	r5, [r4, #20]
 800524a:	42ab      	cmp	r3, r5
 800524c:	d9fa      	bls.n	8005244 <DataStageIn+0x14>
 800524e:	2702      	movs	r7, #2
 8005250:	429d      	cmp	r5, r3
 8005252:	bf28      	it	cs
 8005254:	461d      	movcs	r5, r3
 8005256:	69a2      	ldr	r2, [r4, #24]
 8005258:	4628      	mov	r0, r5
 800525a:	4790      	blx	r2
 800525c:	4680      	mov	r8, r0
 800525e:	2000      	movs	r0, #0
 8005260:	f001 f840 	bl	80062e4 <GetEPTxAddr>
 8005264:	462a      	mov	r2, r5
 8005266:	4601      	mov	r1, r0
 8005268:	4640      	mov	r0, r8
 800526a:	f000 fdcf 	bl	8005e0c <UserToPMABufferCopy>
 800526e:	4629      	mov	r1, r5
 8005270:	2000      	movs	r0, #0
 8005272:	f001 f857 	bl	8006324 <SetEPTxCount>
 8005276:	8a20      	ldrh	r0, [r4, #16]
 8005278:	8a61      	ldrh	r1, [r4, #18]
 800527a:	ebc5 0c00 	rsb	ip, r5, r0
 800527e:	4b15      	ldr	r3, [pc, #84]	; (80052d4 <DataStageIn+0xa4>)
 8005280:	4815      	ldr	r0, [pc, #84]	; (80052d8 <DataStageIn+0xa8>)
 8005282:	186d      	adds	r5, r5, r1
 8005284:	2230      	movs	r2, #48	; 0x30
 8005286:	f44f 5140 	mov.w	r1, #12288	; 0x3000
 800528a:	f8a4 c010 	strh.w	ip, [r4, #16]
 800528e:	8265      	strh	r5, [r4, #18]
 8005290:	801a      	strh	r2, [r3, #0]
 8005292:	6834      	ldr	r4, [r6, #0]
 8005294:	8001      	strh	r1, [r0, #0]
 8005296:	7227      	strb	r7, [r4, #8]
 8005298:	e8bd 81f0 	ldmia.w	sp!, {r4, r5, r6, r7, r8, pc}
 800529c:	4a0f      	ldr	r2, [pc, #60]	; (80052dc <DataStageIn+0xac>)
 800529e:	7811      	ldrb	r1, [r2, #0]
 80052a0:	2901      	cmp	r1, #1
 80052a2:	d006      	beq.n	80052b2 <DataStageIn+0x82>
 80052a4:	4b0b      	ldr	r3, [pc, #44]	; (80052d4 <DataStageIn+0xa4>)
 80052a6:	2707      	movs	r7, #7
 80052a8:	2210      	movs	r2, #16
 80052aa:	801a      	strh	r2, [r3, #0]
 80052ac:	7227      	strb	r7, [r4, #8]
 80052ae:	e8bd 81f0 	ldmia.w	sp!, {r4, r5, r6, r7, r8, pc}
 80052b2:	480b      	ldr	r0, [pc, #44]	; (80052e0 <DataStageIn+0xb0>)
 80052b4:	490b      	ldr	r1, [pc, #44]	; (80052e4 <DataStageIn+0xb4>)
 80052b6:	6804      	ldr	r4, [r0, #0]
 80052b8:	fa1f fc84 	uxth.w	ip, r4
 80052bc:	eb0c 0001 	add.w	r0, ip, r1
 80052c0:	0044      	lsls	r4, r0, #1
 80052c2:	4904      	ldr	r1, [pc, #16]	; (80052d4 <DataStageIn+0xa4>)
 80052c4:	6023      	str	r3, [r4, #0]
 80052c6:	2030      	movs	r0, #48	; 0x30
 80052c8:	6834      	ldr	r4, [r6, #0]
 80052ca:	8008      	strh	r0, [r1, #0]
 80052cc:	7013      	strb	r3, [r2, #0]
 80052ce:	e7e2      	b.n	8005296 <DataStageIn+0x66>
 80052d0:	2000050c 	.word	0x2000050c
 80052d4:	20000514 	.word	0x20000514
 80052d8:	20000512 	.word	0x20000512
 80052dc:	20000234 	.word	0x20000234
 80052e0:	40005c50 	.word	0x40005c50
 80052e4:	20003002 	.word	0x20003002

080052e8 <Standard_SetConfiguration>:
 80052e8:	b510      	push	{r4, lr}
 80052ea:	4b0b      	ldr	r3, [pc, #44]	; (8005318 <Standard_SetConfiguration+0x30>)
 80052ec:	4a0b      	ldr	r2, [pc, #44]	; (800531c <Standard_SetConfiguration+0x34>)
 80052ee:	681b      	ldr	r3, [r3, #0]
 80052f0:	7851      	ldrb	r1, [r2, #1]
 80052f2:	78da      	ldrb	r2, [r3, #3]
 80052f4:	4291      	cmp	r1, r2
 80052f6:	d305      	bcc.n	8005304 <Standard_SetConfiguration+0x1c>
 80052f8:	7898      	ldrb	r0, [r3, #2]
 80052fa:	b918      	cbnz	r0, 8005304 <Standard_SetConfiguration+0x1c>
 80052fc:	889c      	ldrh	r4, [r3, #4]
 80052fe:	b11c      	cbz	r4, 8005308 <Standard_SetConfiguration+0x20>
 8005300:	2002      	movs	r0, #2
 8005302:	bd10      	pop	{r4, pc}
 8005304:	2002      	movs	r0, #2
 8005306:	bd10      	pop	{r4, pc}
 8005308:	729a      	strb	r2, [r3, #10]
 800530a:	4b05      	ldr	r3, [pc, #20]	; (8005320 <Standard_SetConfiguration+0x38>)
 800530c:	681a      	ldr	r2, [r3, #0]
 800530e:	6851      	ldr	r1, [r2, #4]
 8005310:	4788      	blx	r1
 8005312:	4620      	mov	r0, r4
 8005314:	bd10      	pop	{r4, pc}
 8005316:	bf00      	nop
 8005318:	2000050c 	.word	0x2000050c
 800531c:	200000cc 	.word	0x200000cc
 8005320:	20000508 	.word	0x20000508

08005324 <Standard_SetInterface>:
 8005324:	b538      	push	{r3, r4, r5, lr}
 8005326:	4810      	ldr	r0, [pc, #64]	; (8005368 <Standard_SetInterface+0x44>)
 8005328:	4c10      	ldr	r4, [pc, #64]	; (800536c <Standard_SetInterface+0x48>)
 800532a:	6802      	ldr	r2, [r0, #0]
 800532c:	6821      	ldr	r1, [r4, #0]
 800532e:	6993      	ldr	r3, [r2, #24]
 8005330:	7948      	ldrb	r0, [r1, #5]
 8005332:	78c9      	ldrb	r1, [r1, #3]
 8005334:	4798      	blx	r3
 8005336:	6823      	ldr	r3, [r4, #0]
 8005338:	7a9a      	ldrb	r2, [r3, #10]
 800533a:	b18a      	cbz	r2, 8005360 <Standard_SetInterface+0x3c>
 800533c:	b980      	cbnz	r0, 8005360 <Standard_SetInterface+0x3c>
 800533e:	7919      	ldrb	r1, [r3, #4]
 8005340:	b971      	cbnz	r1, 8005360 <Standard_SetInterface+0x3c>
 8005342:	789d      	ldrb	r5, [r3, #2]
 8005344:	b975      	cbnz	r5, 8005364 <Standard_SetInterface+0x40>
 8005346:	4b0a      	ldr	r3, [pc, #40]	; (8005370 <Standard_SetInterface+0x4c>)
 8005348:	6819      	ldr	r1, [r3, #0]
 800534a:	68c8      	ldr	r0, [r1, #12]
 800534c:	4780      	blx	r0
 800534e:	6823      	ldr	r3, [r4, #0]
 8005350:	4628      	mov	r0, r5
 8005352:	795a      	ldrb	r2, [r3, #5]
 8005354:	f893 c003 	ldrb.w	ip, [r3, #3]
 8005358:	72da      	strb	r2, [r3, #11]
 800535a:	f883 c00c 	strb.w	ip, [r3, #12]
 800535e:	bd38      	pop	{r3, r4, r5, pc}
 8005360:	2002      	movs	r0, #2
 8005362:	bd38      	pop	{r3, r4, r5, pc}
 8005364:	2002      	movs	r0, #2
 8005366:	bd38      	pop	{r3, r4, r5, pc}
 8005368:	200004e4 	.word	0x200004e4
 800536c:	2000050c 	.word	0x2000050c
 8005370:	20000508 	.word	0x20000508

08005374 <Standard_ClearFeature>:
 8005374:	b538      	push	{r3, r4, r5, lr}
 8005376:	4b3a      	ldr	r3, [pc, #232]	; (8005460 <Standard_ClearFeature+0xec>)
 8005378:	681b      	ldr	r3, [r3, #0]
 800537a:	7818      	ldrb	r0, [r3, #0]
 800537c:	f010 007f 	ands.w	r0, r0, #127	; 0x7f
 8005380:	d035      	beq.n	80053ee <Standard_ClearFeature+0x7a>
 8005382:	2802      	cmp	r0, #2
 8005384:	d001      	beq.n	800538a <Standard_ClearFeature+0x16>
 8005386:	2002      	movs	r0, #2
 8005388:	bd38      	pop	{r3, r4, r5, pc}
 800538a:	885a      	ldrh	r2, [r3, #2]
 800538c:	2a00      	cmp	r2, #0
 800538e:	d12d      	bne.n	80053ec <Standard_ClearFeature+0x78>
 8005390:	7919      	ldrb	r1, [r3, #4]
 8005392:	bb59      	cbnz	r1, 80053ec <Standard_ClearFeature+0x78>
 8005394:	795a      	ldrb	r2, [r3, #5]
 8005396:	f012 0f80 	tst.w	r2, #128	; 0x80
 800539a:	f022 0080 	bic.w	r0, r2, #128	; 0x80
 800539e:	d12b      	bne.n	80053f8 <Standard_ClearFeature+0x84>
 80053a0:	0084      	lsls	r4, r0, #2
 80053a2:	f104 4180 	add.w	r1, r4, #1073741824	; 0x40000000
 80053a6:	f501 4cb8 	add.w	ip, r1, #23552	; 0x5c00
 80053aa:	f8dc 5000 	ldr.w	r5, [ip]
 80053ae:	f405 5540 	and.w	r5, r5, #12288	; 0x3000
 80053b2:	492c      	ldr	r1, [pc, #176]	; (8005464 <Standard_ClearFeature+0xf0>)
 80053b4:	f891 c000 	ldrb.w	ip, [r1]
 80053b8:	4560      	cmp	r0, ip
 80053ba:	d2e4      	bcs.n	8005386 <Standard_ClearFeature+0x12>
 80053bc:	2d00      	cmp	r5, #0
 80053be:	d0e2      	beq.n	8005386 <Standard_ClearFeature+0x12>
 80053c0:	7a9b      	ldrb	r3, [r3, #10]
 80053c2:	b193      	cbz	r3, 80053ea <Standard_ClearFeature+0x76>
 80053c4:	f012 0f80 	tst.w	r2, #128	; 0x80
 80053c8:	d020      	beq.n	800540c <Standard_ClearFeature+0x98>
 80053ca:	f104 4480 	add.w	r4, r4, #1073741824	; 0x40000000
 80053ce:	f504 44b8 	add.w	r4, r4, #23552	; 0x5c00
 80053d2:	6822      	ldr	r2, [r4, #0]
 80053d4:	f002 0e30 	and.w	lr, r2, #48	; 0x30
 80053d8:	f1be 0f10 	cmp.w	lr, #16
 80053dc:	d033      	beq.n	8005446 <Standard_ClearFeature+0xd2>
 80053de:	4b22      	ldr	r3, [pc, #136]	; (8005468 <Standard_ClearFeature+0xf4>)
 80053e0:	6819      	ldr	r1, [r3, #0]
 80053e2:	6948      	ldr	r0, [r1, #20]
 80053e4:	4780      	blx	r0
 80053e6:	2000      	movs	r0, #0
 80053e8:	bd38      	pop	{r3, r4, r5, pc}
 80053ea:	2002      	movs	r0, #2
 80053ec:	bd38      	pop	{r3, r4, r5, pc}
 80053ee:	7a59      	ldrb	r1, [r3, #9]
 80053f0:	f021 0220 	bic.w	r2, r1, #32
 80053f4:	725a      	strb	r2, [r3, #9]
 80053f6:	bd38      	pop	{r3, r4, r5, pc}
 80053f8:	0084      	lsls	r4, r0, #2
 80053fa:	f104 4180 	add.w	r1, r4, #1073741824	; 0x40000000
 80053fe:	f501 4cb8 	add.w	ip, r1, #23552	; 0x5c00
 8005402:	f8dc 5000 	ldr.w	r5, [ip]
 8005406:	f005 0530 	and.w	r5, r5, #48	; 0x30
 800540a:	e7d2      	b.n	80053b2 <Standard_ClearFeature+0x3e>
 800540c:	f104 4380 	add.w	r3, r4, #1073741824	; 0x40000000
 8005410:	f503 44b8 	add.w	r4, r3, #23552	; 0x5c00
 8005414:	6821      	ldr	r1, [r4, #0]
 8005416:	f401 5c40 	and.w	ip, r1, #12288	; 0x3000
 800541a:	f5bc 5f80 	cmp.w	ip, #4096	; 0x1000
 800541e:	d1de      	bne.n	80053de <Standard_ClearFeature+0x6a>
 8005420:	b9d0      	cbnz	r0, 8005458 <Standard_ClearFeature+0xe4>
 8005422:	4a12      	ldr	r2, [pc, #72]	; (800546c <Standard_ClearFeature+0xf8>)
 8005424:	f892 102c 	ldrb.w	r1, [r2, #44]	; 0x2c
 8005428:	f000 ffa2 	bl	8006370 <SetEPRxCount>
 800542c:	6822      	ldr	r2, [r4, #0]
 800542e:	f64b 738f 	movw	r3, #49039	; 0xbf8f
 8005432:	ea02 0103 	and.w	r1, r2, r3
 8005436:	f481 5c40 	eor.w	ip, r1, #12288	; 0x3000
 800543a:	f44c 4e00 	orr.w	lr, ip, #32768	; 0x8000
 800543e:	f04e 0080 	orr.w	r0, lr, #128	; 0x80
 8005442:	6020      	str	r0, [r4, #0]
 8005444:	e7cb      	b.n	80053de <Standard_ClearFeature+0x6a>
 8005446:	b2c4      	uxtb	r4, r0
 8005448:	4620      	mov	r0, r4
 800544a:	f000 fef5 	bl	8006238 <ClearDTOG_TX>
 800544e:	4620      	mov	r0, r4
 8005450:	2130      	movs	r1, #48	; 0x30
 8005452:	f000 fd99 	bl	8005f88 <SetEPTxStatus>
 8005456:	e7c2      	b.n	80053de <Standard_ClearFeature+0x6a>
 8005458:	f000 feda 	bl	8006210 <ClearDTOG_RX>
 800545c:	e7e6      	b.n	800542c <Standard_ClearFeature+0xb8>
 800545e:	bf00      	nop
 8005460:	2000050c 	.word	0x2000050c
 8005464:	200000cc 	.word	0x200000cc
 8005468:	20000508 	.word	0x20000508
 800546c:	200000d0 	.word	0x200000d0

08005470 <Standard_SetEndPointFeature>:
 8005470:	b538      	push	{r3, r4, r5, lr}
 8005472:	4b29      	ldr	r3, [pc, #164]	; (8005518 <Standard_SetEndPointFeature+0xa8>)
 8005474:	6819      	ldr	r1, [r3, #0]
 8005476:	794b      	ldrb	r3, [r1, #5]
 8005478:	f013 0f80 	tst.w	r3, #128	; 0x80
 800547c:	f023 0280 	bic.w	r2, r3, #128	; 0x80
 8005480:	d12b      	bne.n	80054da <Standard_SetEndPointFeature+0x6a>
 8005482:	0090      	lsls	r0, r2, #2
 8005484:	f100 4c80 	add.w	ip, r0, #1073741824	; 0x40000000
 8005488:	f50c 44b8 	add.w	r4, ip, #23552	; 0x5c00
 800548c:	6825      	ldr	r5, [r4, #0]
 800548e:	f405 5540 	and.w	r5, r5, #12288	; 0x3000
 8005492:	4c22      	ldr	r4, [pc, #136]	; (800551c <Standard_SetEndPointFeature+0xac>)
 8005494:	7824      	ldrb	r4, [r4, #0]
 8005496:	42a2      	cmp	r2, r4
 8005498:	d21d      	bcs.n	80054d6 <Standard_SetEndPointFeature+0x66>
 800549a:	884a      	ldrh	r2, [r1, #2]
 800549c:	b9da      	cbnz	r2, 80054d6 <Standard_SetEndPointFeature+0x66>
 800549e:	b1d5      	cbz	r5, 80054d6 <Standard_SetEndPointFeature+0x66>
 80054a0:	7a89      	ldrb	r1, [r1, #10]
 80054a2:	b319      	cbz	r1, 80054ec <Standard_SetEndPointFeature+0x7c>
 80054a4:	f013 0f80 	tst.w	r3, #128	; 0x80
 80054a8:	d022      	beq.n	80054f0 <Standard_SetEndPointFeature+0x80>
 80054aa:	f100 4080 	add.w	r0, r0, #1073741824	; 0x40000000
 80054ae:	f500 40b8 	add.w	r0, r0, #23552	; 0x5c00
 80054b2:	6803      	ldr	r3, [r0, #0]
 80054b4:	f648 71bf 	movw	r1, #36799	; 0x8fbf
 80054b8:	ea03 0201 	and.w	r2, r3, r1
 80054bc:	f082 0c10 	eor.w	ip, r2, #16
 80054c0:	f44c 4e00 	orr.w	lr, ip, #32768	; 0x8000
 80054c4:	f04e 0380 	orr.w	r3, lr, #128	; 0x80
 80054c8:	6003      	str	r3, [r0, #0]
 80054ca:	4815      	ldr	r0, [pc, #84]	; (8005520 <Standard_SetEndPointFeature+0xb0>)
 80054cc:	6803      	ldr	r3, [r0, #0]
 80054ce:	6999      	ldr	r1, [r3, #24]
 80054d0:	4788      	blx	r1
 80054d2:	2000      	movs	r0, #0
 80054d4:	bd38      	pop	{r3, r4, r5, pc}
 80054d6:	2002      	movs	r0, #2
 80054d8:	bd38      	pop	{r3, r4, r5, pc}
 80054da:	0090      	lsls	r0, r2, #2
 80054dc:	f100 4c80 	add.w	ip, r0, #1073741824	; 0x40000000
 80054e0:	f50c 44b8 	add.w	r4, ip, #23552	; 0x5c00
 80054e4:	6825      	ldr	r5, [r4, #0]
 80054e6:	f005 0530 	and.w	r5, r5, #48	; 0x30
 80054ea:	e7d2      	b.n	8005492 <Standard_SetEndPointFeature+0x22>
 80054ec:	2002      	movs	r0, #2
 80054ee:	bd38      	pop	{r3, r4, r5, pc}
 80054f0:	f100 4e80 	add.w	lr, r0, #1073741824	; 0x40000000
 80054f4:	f50e 4eb8 	add.w	lr, lr, #23552	; 0x5c00
 80054f8:	f8de 2000 	ldr.w	r2, [lr]
 80054fc:	f64b 708f 	movw	r0, #49039	; 0xbf8f
 8005500:	ea02 0300 	and.w	r3, r2, r0
 8005504:	f483 5180 	eor.w	r1, r3, #4096	; 0x1000
 8005508:	f441 4c00 	orr.w	ip, r1, #32768	; 0x8000
 800550c:	f04c 0280 	orr.w	r2, ip, #128	; 0x80
 8005510:	f8ce 2000 	str.w	r2, [lr]
 8005514:	e7d9      	b.n	80054ca <Standard_SetEndPointFeature+0x5a>
 8005516:	bf00      	nop
 8005518:	2000050c 	.word	0x2000050c
 800551c:	200000cc 	.word	0x200000cc
 8005520:	20000508 	.word	0x20000508

08005524 <Standard_SetDeviceFeature>:
 8005524:	b508      	push	{r3, lr}
 8005526:	4806      	ldr	r0, [pc, #24]	; (8005540 <Standard_SetDeviceFeature+0x1c>)
 8005528:	4906      	ldr	r1, [pc, #24]	; (8005544 <Standard_SetDeviceFeature+0x20>)
 800552a:	6803      	ldr	r3, [r0, #0]
 800552c:	f893 c009 	ldrb.w	ip, [r3, #9]
 8005530:	f04c 0220 	orr.w	r2, ip, #32
 8005534:	725a      	strb	r2, [r3, #9]
 8005536:	6808      	ldr	r0, [r1, #0]
 8005538:	69c3      	ldr	r3, [r0, #28]
 800553a:	4798      	blx	r3
 800553c:	2000      	movs	r0, #0
 800553e:	bd08      	pop	{r3, pc}
 8005540:	2000050c 	.word	0x2000050c
 8005544:	20000508 	.word	0x20000508

08005548 <Standard_GetDescriptorData>:
 8005548:	4b05      	ldr	r3, [pc, #20]	; (8005560 <Standard_GetDescriptorData+0x18>)
 800554a:	681b      	ldr	r3, [r3, #0]
 800554c:	8a5a      	ldrh	r2, [r3, #18]
 800554e:	b110      	cbz	r0, 8005556 <Standard_GetDescriptorData+0xe>
 8005550:	6808      	ldr	r0, [r1, #0]
 8005552:	1880      	adds	r0, r0, r2
 8005554:	4770      	bx	lr
 8005556:	8889      	ldrh	r1, [r1, #4]
 8005558:	1a8a      	subs	r2, r1, r2
 800555a:	821a      	strh	r2, [r3, #16]
 800555c:	4770      	bx	lr
 800555e:	bf00      	nop
 8005560:	2000050c 	.word	0x2000050c

08005564 <Post0_Process>:
 8005564:	b508      	push	{r3, lr}
 8005566:	490d      	ldr	r1, [pc, #52]	; (800559c <Post0_Process+0x38>)
 8005568:	2000      	movs	r0, #0
 800556a:	f891 102c 	ldrb.w	r1, [r1, #44]	; 0x2c
 800556e:	f000 feff 	bl	8006370 <SetEPRxCount>
 8005572:	480b      	ldr	r0, [pc, #44]	; (80055a0 <Post0_Process+0x3c>)
 8005574:	6803      	ldr	r3, [r0, #0]
 8005576:	7a18      	ldrb	r0, [r3, #8]
 8005578:	2808      	cmp	r0, #8
 800557a:	d107      	bne.n	800558c <Post0_Process+0x28>
 800557c:	4909      	ldr	r1, [pc, #36]	; (80055a4 <Post0_Process+0x40>)
 800557e:	4b0a      	ldr	r3, [pc, #40]	; (80055a8 <Post0_Process+0x44>)
 8005580:	f44f 5c80 	mov.w	ip, #4096	; 0x1000
 8005584:	2210      	movs	r2, #16
 8005586:	f8a1 c000 	strh.w	ip, [r1]
 800558a:	801a      	strh	r2, [r3, #0]
 800558c:	f1a0 0009 	sub.w	r0, r0, #9
 8005590:	f1d0 0e00 	rsbs	lr, r0, #0
 8005594:	eb4e 0000 	adc.w	r0, lr, r0
 8005598:	bd08      	pop	{r3, pc}
 800559a:	bf00      	nop
 800559c:	200000d0 	.word	0x200000d0
 80055a0:	2000050c 	.word	0x2000050c
 80055a4:	20000512 	.word	0x20000512
 80055a8:	20000514 	.word	0x20000514

080055ac <Out0_Process>:
 80055ac:	e92d 41f0 	stmdb	sp!, {r4, r5, r6, r7, r8, lr}
 80055b0:	4d2f      	ldr	r5, [pc, #188]	; (8005670 <Out0_Process+0xc4>)
 80055b2:	682c      	ldr	r4, [r5, #0]
 80055b4:	7a23      	ldrb	r3, [r4, #8]
 80055b6:	2b02      	cmp	r3, #2
 80055b8:	d007      	beq.n	80055ca <Out0_Process+0x1e>
 80055ba:	2b04      	cmp	r3, #4
 80055bc:	d005      	beq.n	80055ca <Out0_Process+0x1e>
 80055be:	2b03      	cmp	r3, #3
 80055c0:	d008      	beq.n	80055d4 <Out0_Process+0x28>
 80055c2:	2b05      	cmp	r3, #5
 80055c4:	d006      	beq.n	80055d4 <Out0_Process+0x28>
 80055c6:	2b07      	cmp	r3, #7
 80055c8:	d042      	beq.n	8005650 <Out0_Process+0xa4>
 80055ca:	2308      	movs	r3, #8
 80055cc:	7223      	strb	r3, [r4, #8]
 80055ce:	e8bd 41f0 	ldmia.w	sp!, {r4, r5, r6, r7, r8, lr}
 80055d2:	e7c7      	b.n	8005564 <Post0_Process>
 80055d4:	69a3      	ldr	r3, [r4, #24]
 80055d6:	8a22      	ldrh	r2, [r4, #16]
 80055d8:	b1b3      	cbz	r3, 8005608 <Out0_Process+0x5c>
 80055da:	b31a      	cbz	r2, 8005624 <Out0_Process+0x78>
 80055dc:	8aa6      	ldrh	r6, [r4, #20]
 80055de:	4296      	cmp	r6, r2
 80055e0:	bf28      	it	cs
 80055e2:	4616      	movcs	r6, r2
 80055e4:	4630      	mov	r0, r6
 80055e6:	4798      	blx	r3
 80055e8:	8a21      	ldrh	r1, [r4, #16]
 80055ea:	4607      	mov	r7, r0
 80055ec:	8a60      	ldrh	r0, [r4, #18]
 80055ee:	1b8a      	subs	r2, r1, r6
 80055f0:	1833      	adds	r3, r6, r0
 80055f2:	8222      	strh	r2, [r4, #16]
 80055f4:	8263      	strh	r3, [r4, #18]
 80055f6:	2000      	movs	r0, #0
 80055f8:	f000 fe84 	bl	8006304 <GetEPRxAddr>
 80055fc:	4632      	mov	r2, r6
 80055fe:	4601      	mov	r1, r0
 8005600:	4638      	mov	r0, r7
 8005602:	f000 fc37 	bl	8005e74 <PMAToUserBufferCopy>
 8005606:	8a22      	ldrh	r2, [r4, #16]
 8005608:	b162      	cbz	r2, 8005624 <Out0_Process+0x78>
 800560a:	4a1a      	ldr	r2, [pc, #104]	; (8005674 <Out0_Process+0xc8>)
 800560c:	2000      	movs	r0, #0
 800560e:	f44f 5c40 	mov.w	ip, #12288	; 0x3000
 8005612:	4601      	mov	r1, r0
 8005614:	f8a2 c000 	strh.w	ip, [r2]
 8005618:	f000 fe84 	bl	8006324 <SetEPTxCount>
 800561c:	4b16      	ldr	r3, [pc, #88]	; (8005678 <Out0_Process+0xcc>)
 800561e:	2030      	movs	r0, #48	; 0x30
 8005620:	8a22      	ldrh	r2, [r4, #16]
 8005622:	8018      	strh	r0, [r3, #0]
 8005624:	8aa4      	ldrh	r4, [r4, #20]
 8005626:	4294      	cmp	r4, r2
 8005628:	d919      	bls.n	800565e <Out0_Process+0xb2>
 800562a:	b9e2      	cbnz	r2, 8005666 <Out0_Process+0xba>
 800562c:	6828      	ldr	r0, [r5, #0]
 800562e:	2406      	movs	r4, #6
 8005630:	4b12      	ldr	r3, [pc, #72]	; (800567c <Out0_Process+0xd0>)
 8005632:	7204      	strb	r4, [r0, #8]
 8005634:	6819      	ldr	r1, [r3, #0]
 8005636:	4812      	ldr	r0, [pc, #72]	; (8005680 <Out0_Process+0xd4>)
 8005638:	fa1f fc81 	uxth.w	ip, r1
 800563c:	eb0c 0400 	add.w	r4, ip, r0
 8005640:	0063      	lsls	r3, r4, #1
 8005642:	601a      	str	r2, [r3, #0]
 8005644:	682c      	ldr	r4, [r5, #0]
 8005646:	480c      	ldr	r0, [pc, #48]	; (8005678 <Out0_Process+0xcc>)
 8005648:	2130      	movs	r1, #48	; 0x30
 800564a:	7a23      	ldrb	r3, [r4, #8]
 800564c:	8001      	strh	r1, [r0, #0]
 800564e:	e7bd      	b.n	80055cc <Out0_Process+0x20>
 8005650:	4b0c      	ldr	r3, [pc, #48]	; (8005684 <Out0_Process+0xd8>)
 8005652:	6819      	ldr	r1, [r3, #0]
 8005654:	68ca      	ldr	r2, [r1, #12]
 8005656:	4790      	blx	r2
 8005658:	682c      	ldr	r4, [r5, #0]
 800565a:	2308      	movs	r3, #8
 800565c:	e7b6      	b.n	80055cc <Out0_Process+0x20>
 800565e:	682c      	ldr	r4, [r5, #0]
 8005660:	2303      	movs	r3, #3
 8005662:	7223      	strb	r3, [r4, #8]
 8005664:	e7b2      	b.n	80055cc <Out0_Process+0x20>
 8005666:	682c      	ldr	r4, [r5, #0]
 8005668:	2305      	movs	r3, #5
 800566a:	7223      	strb	r3, [r4, #8]
 800566c:	e7ae      	b.n	80055cc <Out0_Process+0x20>
 800566e:	bf00      	nop
 8005670:	2000050c 	.word	0x2000050c
 8005674:	20000512 	.word	0x20000512
 8005678:	20000514 	.word	0x20000514
 800567c:	40005c50 	.word	0x40005c50
 8005680:	20003002 	.word	0x20003002
 8005684:	200004e4 	.word	0x200004e4

08005688 <Setup0_Process>:
 8005688:	b5f0      	push	{r4, r5, r6, r7, lr}
 800568a:	4a9a      	ldr	r2, [pc, #616]	; (80058f4 <Setup0_Process+0x26c>)
 800568c:	4c9a      	ldr	r4, [pc, #616]	; (80058f8 <Setup0_Process+0x270>)
 800568e:	6811      	ldr	r1, [r2, #0]
 8005690:	6822      	ldr	r2, [r4, #0]
 8005692:	4b9a      	ldr	r3, [pc, #616]	; (80058fc <Setup0_Process+0x274>)
 8005694:	fa1f fc81 	uxth.w	ip, r1
 8005698:	7a11      	ldrb	r1, [r2, #8]
 800569a:	eb0c 0003 	add.w	r0, ip, r3
 800569e:	0043      	lsls	r3, r0, #1
 80056a0:	2909      	cmp	r1, #9
 80056a2:	b083      	sub	sp, #12
 80056a4:	681b      	ldr	r3, [r3, #0]
 80056a6:	d04d      	beq.n	8005744 <Setup0_Process+0xbc>
 80056a8:	b299      	uxth	r1, r3
 80056aa:	f101 5c00 	add.w	ip, r1, #536870912	; 0x20000000
 80056ae:	f50c 5640 	add.w	r6, ip, #12288	; 0x3000
 80056b2:	0070      	lsls	r0, r6, #1
 80056b4:	4605      	mov	r5, r0
 80056b6:	f815 3b01 	ldrb.w	r3, [r5], #1
 80056ba:	f100 060a 	add.w	r6, r0, #10
 80056be:	7013      	strb	r3, [r2, #0]
 80056c0:	f890 e001 	ldrb.w	lr, [r0, #1]
 80056c4:	6827      	ldr	r7, [r4, #0]
 80056c6:	f887 e001 	strb.w	lr, [r7, #1]
 80056ca:	f8b5 0003 	ldrh.w	r0, [r5, #3]
 80056ce:	6827      	ldr	r7, [r4, #0]
 80056d0:	f001 f832 	bl	8006738 <ByteSwap>
 80056d4:	8078      	strh	r0, [r7, #2]
 80056d6:	f8b5 0007 	ldrh.w	r0, [r5, #7]
 80056da:	6825      	ldr	r5, [r4, #0]
 80056dc:	f001 f82c 	bl	8006738 <ByteSwap>
 80056e0:	80a8      	strh	r0, [r5, #4]
 80056e2:	6822      	ldr	r2, [r4, #0]
 80056e4:	8873      	ldrh	r3, [r6, #2]
 80056e6:	2001      	movs	r0, #1
 80056e8:	80d3      	strh	r3, [r2, #6]
 80056ea:	7210      	strb	r0, [r2, #8]
 80056ec:	2b00      	cmp	r3, #0
 80056ee:	d02e      	beq.n	800574e <Setup0_Process+0xc6>
 80056f0:	7851      	ldrb	r1, [r2, #1]
 80056f2:	2906      	cmp	r1, #6
 80056f4:	f000 808a 	beq.w	800580c <Setup0_Process+0x184>
 80056f8:	2900      	cmp	r1, #0
 80056fa:	d147      	bne.n	800578c <Setup0_Process+0x104>
 80056fc:	8850      	ldrh	r0, [r2, #2]
 80056fe:	2800      	cmp	r0, #0
 8005700:	f000 809b 	beq.w	800583a <Setup0_Process+0x1b2>
 8005704:	4d7e      	ldr	r5, [pc, #504]	; (8005900 <Setup0_Process+0x278>)
 8005706:	682b      	ldr	r3, [r5, #0]
 8005708:	4608      	mov	r0, r1
 800570a:	691a      	ldr	r2, [r3, #16]
 800570c:	4790      	blx	r2
 800570e:	2803      	cmp	r0, #3
 8005710:	f000 8106 	beq.w	8005920 <Setup0_Process+0x298>
 8005714:	6823      	ldr	r3, [r4, #0]
 8005716:	f64f 7cff 	movw	ip, #65535	; 0xffff
 800571a:	8a1a      	ldrh	r2, [r3, #16]
 800571c:	4562      	cmp	r2, ip
 800571e:	d048      	beq.n	80057b2 <Setup0_Process+0x12a>
 8005720:	2802      	cmp	r0, #2
 8005722:	d02d      	beq.n	8005780 <Setup0_Process+0xf8>
 8005724:	2a00      	cmp	r2, #0
 8005726:	d02b      	beq.n	8005780 <Setup0_Process+0xf8>
 8005728:	f993 e000 	ldrsb.w	lr, [r3]
 800572c:	f1be 0f00 	cmp.w	lr, #0
 8005730:	f2c0 80ba 	blt.w	80058a8 <Setup0_Process+0x220>
 8005734:	2103      	movs	r1, #3
 8005736:	7219      	strb	r1, [r3, #8]
 8005738:	4b72      	ldr	r3, [pc, #456]	; (8005904 <Setup0_Process+0x27c>)
 800573a:	f44f 5e40 	mov.w	lr, #12288	; 0x3000
 800573e:	f8a3 e000 	strh.w	lr, [r3]
 8005742:	e01f      	b.n	8005784 <Setup0_Process+0xfc>
 8005744:	88d3      	ldrh	r3, [r2, #6]
 8005746:	2001      	movs	r0, #1
 8005748:	7210      	strb	r0, [r2, #8]
 800574a:	2b00      	cmp	r3, #0
 800574c:	d1d0      	bne.n	80056f0 <Setup0_Process+0x68>
 800574e:	7813      	ldrb	r3, [r2, #0]
 8005750:	7855      	ldrb	r5, [r2, #1]
 8005752:	f013 037f 	ands.w	r3, r3, #127	; 0x7f
 8005756:	d02f      	beq.n	80057b8 <Setup0_Process+0x130>
 8005758:	2b01      	cmp	r3, #1
 800575a:	f000 80bb 	beq.w	80058d4 <Setup0_Process+0x24c>
 800575e:	2b02      	cmp	r3, #2
 8005760:	f000 80f6 	beq.w	8005950 <Setup0_Process+0x2c8>
 8005764:	4a66      	ldr	r2, [pc, #408]	; (8005900 <Setup0_Process+0x278>)
 8005766:	4628      	mov	r0, r5
 8005768:	6811      	ldr	r1, [r2, #0]
 800576a:	694b      	ldr	r3, [r1, #20]
 800576c:	4798      	blx	r3
 800576e:	2803      	cmp	r0, #3
 8005770:	f000 8119 	beq.w	80059a6 <Setup0_Process+0x31e>
 8005774:	2800      	cmp	r0, #0
 8005776:	d038      	beq.n	80057ea <Setup0_Process+0x162>
 8005778:	6822      	ldr	r2, [r4, #0]
 800577a:	2308      	movs	r3, #8
 800577c:	7213      	strb	r3, [r2, #8]
 800577e:	e001      	b.n	8005784 <Setup0_Process+0xfc>
 8005780:	2108      	movs	r1, #8
 8005782:	7219      	strb	r1, [r3, #8]
 8005784:	b003      	add	sp, #12
 8005786:	e8bd 40f0 	ldmia.w	sp!, {r4, r5, r6, r7, lr}
 800578a:	e6eb      	b.n	8005564 <Post0_Process>
 800578c:	2908      	cmp	r1, #8
 800578e:	d167      	bne.n	8005860 <Setup0_Process+0x1d8>
 8005790:	7810      	ldrb	r0, [r2, #0]
 8005792:	f010 0f7f 	tst.w	r0, #127	; 0x7f
 8005796:	d1b5      	bne.n	8005704 <Setup0_Process+0x7c>
 8005798:	4b5b      	ldr	r3, [pc, #364]	; (8005908 <Setup0_Process+0x280>)
 800579a:	2100      	movs	r1, #0
 800579c:	6193      	str	r3, [r2, #24]
 800579e:	8251      	strh	r1, [r2, #18]
 80057a0:	2000      	movs	r0, #0
 80057a2:	4798      	blx	r3
 80057a4:	6823      	ldr	r3, [r4, #0]
 80057a6:	f64f 7cff 	movw	ip, #65535	; 0xffff
 80057aa:	8a1a      	ldrh	r2, [r3, #16]
 80057ac:	2000      	movs	r0, #0
 80057ae:	4562      	cmp	r2, ip
 80057b0:	d1b6      	bne.n	8005720 <Setup0_Process+0x98>
 80057b2:	2209      	movs	r2, #9
 80057b4:	721a      	strb	r2, [r3, #8]
 80057b6:	e7e5      	b.n	8005784 <Setup0_Process+0xfc>
 80057b8:	2d09      	cmp	r5, #9
 80057ba:	f000 80ea 	beq.w	8005992 <Setup0_Process+0x30a>
 80057be:	2d05      	cmp	r5, #5
 80057c0:	f000 80b6 	beq.w	8005930 <Setup0_Process+0x2a8>
 80057c4:	2d03      	cmp	r5, #3
 80057c6:	f000 80cf 	beq.w	8005968 <Setup0_Process+0x2e0>
 80057ca:	2d01      	cmp	r5, #1
 80057cc:	d1ca      	bne.n	8005764 <Setup0_Process+0xdc>
 80057ce:	78d1      	ldrb	r1, [r2, #3]
 80057d0:	2901      	cmp	r1, #1
 80057d2:	d1c7      	bne.n	8005764 <Setup0_Process+0xdc>
 80057d4:	8890      	ldrh	r0, [r2, #4]
 80057d6:	2800      	cmp	r0, #0
 80057d8:	d1c4      	bne.n	8005764 <Setup0_Process+0xdc>
 80057da:	7a52      	ldrb	r2, [r2, #9]
 80057dc:	f012 0f20 	tst.w	r2, #32
 80057e0:	d0c0      	beq.n	8005764 <Setup0_Process+0xdc>
 80057e2:	f7ff fdc7 	bl	8005374 <Standard_ClearFeature>
 80057e6:	2800      	cmp	r0, #0
 80057e8:	d1bc      	bne.n	8005764 <Setup0_Process+0xdc>
 80057ea:	4a42      	ldr	r2, [pc, #264]	; (80058f4 <Setup0_Process+0x26c>)
 80057ec:	4b47      	ldr	r3, [pc, #284]	; (800590c <Setup0_Process+0x284>)
 80057ee:	6811      	ldr	r1, [r2, #0]
 80057f0:	fa1f fc81 	uxth.w	ip, r1
 80057f4:	eb0c 0003 	add.w	r0, ip, r3
 80057f8:	0042      	lsls	r2, r0, #1
 80057fa:	2100      	movs	r1, #0
 80057fc:	6011      	str	r1, [r2, #0]
 80057fe:	4b44      	ldr	r3, [pc, #272]	; (8005910 <Setup0_Process+0x288>)
 8005800:	6822      	ldr	r2, [r4, #0]
 8005802:	2030      	movs	r0, #48	; 0x30
 8005804:	8018      	strh	r0, [r3, #0]
 8005806:	2306      	movs	r3, #6
 8005808:	7213      	strb	r3, [r2, #8]
 800580a:	e7bb      	b.n	8005784 <Setup0_Process+0xfc>
 800580c:	7815      	ldrb	r5, [r2, #0]
 800580e:	f015 0f7f 	tst.w	r5, #127	; 0x7f
 8005812:	f47f af77 	bne.w	8005704 <Setup0_Process+0x7c>
 8005816:	7893      	ldrb	r3, [r2, #2]
 8005818:	2b01      	cmp	r3, #1
 800581a:	f000 80c0 	beq.w	800599e <Setup0_Process+0x316>
 800581e:	2b02      	cmp	r3, #2
 8005820:	f000 8082 	beq.w	8005928 <Setup0_Process+0x2a0>
 8005824:	2b03      	cmp	r3, #3
 8005826:	f47f af6d 	bne.w	8005704 <Setup0_Process+0x7c>
 800582a:	4d35      	ldr	r5, [pc, #212]	; (8005900 <Setup0_Process+0x278>)
 800582c:	6829      	ldr	r1, [r5, #0]
 800582e:	6a4b      	ldr	r3, [r1, #36]	; 0x24
 8005830:	2b00      	cmp	r3, #0
 8005832:	d1b2      	bne.n	800579a <Setup0_Process+0x112>
 8005834:	6820      	ldr	r0, [r4, #0]
 8005836:	7841      	ldrb	r1, [r0, #1]
 8005838:	e765      	b.n	8005706 <Setup0_Process+0x7e>
 800583a:	6853      	ldr	r3, [r2, #4]
 800583c:	f423 4e7f 	bic.w	lr, r3, #65280	; 0xff00
 8005840:	f5be 3f00 	cmp.w	lr, #131072	; 0x20000
 8005844:	f47f af5e 	bne.w	8005704 <Setup0_Process+0x7c>
 8005848:	f892 c000 	ldrb.w	ip, [r2]
 800584c:	f01c 037f 	ands.w	r3, ip, #127	; 0x7f
 8005850:	f040 80ad 	bne.w	80059ae <Setup0_Process+0x326>
 8005854:	8890      	ldrh	r0, [r2, #4]
 8005856:	2800      	cmp	r0, #0
 8005858:	f47f af54 	bne.w	8005704 <Setup0_Process+0x7c>
 800585c:	4b2d      	ldr	r3, [pc, #180]	; (8005914 <Setup0_Process+0x28c>)
 800585e:	e79c      	b.n	800579a <Setup0_Process+0x112>
 8005860:	290a      	cmp	r1, #10
 8005862:	f47f af4f 	bne.w	8005704 <Setup0_Process+0x7c>
 8005866:	f892 c000 	ldrb.w	ip, [r2]
 800586a:	f00c 037f 	and.w	r3, ip, #127	; 0x7f
 800586e:	2b01      	cmp	r3, #1
 8005870:	f47f af48 	bne.w	8005704 <Setup0_Process+0x7c>
 8005874:	7a95      	ldrb	r5, [r2, #10]
 8005876:	2d00      	cmp	r5, #0
 8005878:	f43f af44 	beq.w	8005704 <Setup0_Process+0x7c>
 800587c:	8856      	ldrh	r6, [r2, #2]
 800587e:	2e00      	cmp	r6, #0
 8005880:	f47f af40 	bne.w	8005704 <Setup0_Process+0x7c>
 8005884:	6850      	ldr	r0, [r2, #4]
 8005886:	f420 4e7f 	bic.w	lr, r0, #65280	; 0xff00
 800588a:	f5be 3f80 	cmp.w	lr, #65536	; 0x10000
 800588e:	f47f af39 	bne.w	8005704 <Setup0_Process+0x7c>
 8005892:	4d1b      	ldr	r5, [pc, #108]	; (8005900 <Setup0_Process+0x278>)
 8005894:	7950      	ldrb	r0, [r2, #5]
 8005896:	682b      	ldr	r3, [r5, #0]
 8005898:	4631      	mov	r1, r6
 800589a:	699a      	ldr	r2, [r3, #24]
 800589c:	4790      	blx	r2
 800589e:	2800      	cmp	r0, #0
 80058a0:	d1c8      	bne.n	8005834 <Setup0_Process+0x1ac>
 80058a2:	6822      	ldr	r2, [r4, #0]
 80058a4:	4b1c      	ldr	r3, [pc, #112]	; (8005918 <Setup0_Process+0x290>)
 80058a6:	e778      	b.n	800579a <Setup0_Process+0x112>
 80058a8:	88d9      	ldrh	r1, [r3, #6]
 80058aa:	9101      	str	r1, [sp, #4]
 80058ac:	9801      	ldr	r0, [sp, #4]
 80058ae:	4282      	cmp	r2, r0
 80058b0:	d818      	bhi.n	80058e4 <Setup0_Process+0x25c>
 80058b2:	428a      	cmp	r2, r1
 80058b4:	f080 809d 	bcs.w	80059f2 <Setup0_Process+0x36a>
 80058b8:	4911      	ldr	r1, [pc, #68]	; (8005900 <Setup0_Process+0x278>)
 80058ba:	6808      	ldr	r0, [r1, #0]
 80058bc:	f890 102c 	ldrb.w	r1, [r0, #44]	; 0x2c
 80058c0:	428a      	cmp	r2, r1
 80058c2:	f080 809b 	bcs.w	80059fc <Setup0_Process+0x374>
 80058c6:	4a15      	ldr	r2, [pc, #84]	; (800591c <Setup0_Process+0x294>)
 80058c8:	2000      	movs	r0, #0
 80058ca:	7010      	strb	r0, [r2, #0]
 80058cc:	8299      	strh	r1, [r3, #20]
 80058ce:	f7ff fcaf 	bl	8005230 <DataStageIn>
 80058d2:	e757      	b.n	8005784 <Setup0_Process+0xfc>
 80058d4:	2d0b      	cmp	r5, #11
 80058d6:	f47f af45 	bne.w	8005764 <Setup0_Process+0xdc>
 80058da:	f7ff fd23 	bl	8005324 <Standard_SetInterface>
 80058de:	2800      	cmp	r0, #0
 80058e0:	d083      	beq.n	80057ea <Setup0_Process+0x162>
 80058e2:	e73f      	b.n	8005764 <Setup0_Process+0xdc>
 80058e4:	4806      	ldr	r0, [pc, #24]	; (8005900 <Setup0_Process+0x278>)
 80058e6:	9901      	ldr	r1, [sp, #4]
 80058e8:	6802      	ldr	r2, [r0, #0]
 80058ea:	8219      	strh	r1, [r3, #16]
 80058ec:	f892 102c 	ldrb.w	r1, [r2, #44]	; 0x2c
 80058f0:	e7ec      	b.n	80058cc <Setup0_Process+0x244>
 80058f2:	bf00      	nop
 80058f4:	40005c50 	.word	0x40005c50
 80058f8:	2000050c 	.word	0x2000050c
 80058fc:	20003004 	.word	0x20003004
 8005900:	200004e4 	.word	0x200004e4
 8005904:	20000512 	.word	0x20000512
 8005908:	08005125 	.word	0x08005125
 800590c:	20003002 	.word	0x20003002
 8005910:	20000514 	.word	0x20000514
 8005914:	08005175 	.word	0x08005175
 8005918:	0800514d 	.word	0x0800514d
 800591c:	20000234 	.word	0x20000234
 8005920:	6820      	ldr	r0, [r4, #0]
 8005922:	2109      	movs	r1, #9
 8005924:	7201      	strb	r1, [r0, #8]
 8005926:	e72d      	b.n	8005784 <Setup0_Process+0xfc>
 8005928:	4d46      	ldr	r5, [pc, #280]	; (8005a44 <Setup0_Process+0x3bc>)
 800592a:	682b      	ldr	r3, [r5, #0]
 800592c:	6a1b      	ldr	r3, [r3, #32]
 800592e:	e77f      	b.n	8005830 <Setup0_Process+0x1a8>
 8005930:	f992 c003 	ldrsb.w	ip, [r2, #3]
 8005934:	f1bc 0f00 	cmp.w	ip, #0
 8005938:	db07      	blt.n	800594a <Setup0_Process+0x2c2>
 800593a:	7891      	ldrb	r1, [r2, #2]
 800593c:	b929      	cbnz	r1, 800594a <Setup0_Process+0x2c2>
 800593e:	8890      	ldrh	r0, [r2, #4]
 8005940:	b918      	cbnz	r0, 800594a <Setup0_Process+0x2c2>
 8005942:	7a93      	ldrb	r3, [r2, #10]
 8005944:	2b00      	cmp	r3, #0
 8005946:	f43f af50 	beq.w	80057ea <Setup0_Process+0x162>
 800594a:	2308      	movs	r3, #8
 800594c:	7213      	strb	r3, [r2, #8]
 800594e:	e719      	b.n	8005784 <Setup0_Process+0xfc>
 8005950:	2d01      	cmp	r5, #1
 8005952:	f43f af46 	beq.w	80057e2 <Setup0_Process+0x15a>
 8005956:	2d03      	cmp	r5, #3
 8005958:	f47f af04 	bne.w	8005764 <Setup0_Process+0xdc>
 800595c:	f7ff fd88 	bl	8005470 <Standard_SetEndPointFeature>
 8005960:	2800      	cmp	r0, #0
 8005962:	f43f af42 	beq.w	80057ea <Setup0_Process+0x162>
 8005966:	e6fd      	b.n	8005764 <Setup0_Process+0xdc>
 8005968:	f892 e003 	ldrb.w	lr, [r2, #3]
 800596c:	f1be 0f01 	cmp.w	lr, #1
 8005970:	f47f aef8 	bne.w	8005764 <Setup0_Process+0xdc>
 8005974:	8891      	ldrh	r1, [r2, #4]
 8005976:	2900      	cmp	r1, #0
 8005978:	f47f aef4 	bne.w	8005764 <Setup0_Process+0xdc>
 800597c:	f892 e009 	ldrb.w	lr, [r2, #9]
 8005980:	4b31      	ldr	r3, [pc, #196]	; (8005a48 <Setup0_Process+0x3c0>)
 8005982:	f04e 0c20 	orr.w	ip, lr, #32
 8005986:	f882 c009 	strb.w	ip, [r2, #9]
 800598a:	6818      	ldr	r0, [r3, #0]
 800598c:	69c2      	ldr	r2, [r0, #28]
 800598e:	4790      	blx	r2
 8005990:	e72b      	b.n	80057ea <Setup0_Process+0x162>
 8005992:	f7ff fca9 	bl	80052e8 <Standard_SetConfiguration>
 8005996:	2800      	cmp	r0, #0
 8005998:	f43f af27 	beq.w	80057ea <Setup0_Process+0x162>
 800599c:	e6e2      	b.n	8005764 <Setup0_Process+0xdc>
 800599e:	4d29      	ldr	r5, [pc, #164]	; (8005a44 <Setup0_Process+0x3bc>)
 80059a0:	6828      	ldr	r0, [r5, #0]
 80059a2:	69c3      	ldr	r3, [r0, #28]
 80059a4:	e744      	b.n	8005830 <Setup0_Process+0x1a8>
 80059a6:	6822      	ldr	r2, [r4, #0]
 80059a8:	2309      	movs	r3, #9
 80059aa:	7213      	strb	r3, [r2, #8]
 80059ac:	e6ea      	b.n	8005784 <Setup0_Process+0xfc>
 80059ae:	2b01      	cmp	r3, #1
 80059b0:	d02f      	beq.n	8005a12 <Setup0_Process+0x38a>
 80059b2:	2b02      	cmp	r3, #2
 80059b4:	f47f aea6 	bne.w	8005704 <Setup0_Process+0x7c>
 80059b8:	7953      	ldrb	r3, [r2, #5]
 80059ba:	f013 0f80 	tst.w	r3, #128	; 0x80
 80059be:	f003 000f 	and.w	r0, r3, #15
 80059c2:	d135      	bne.n	8005a30 <Setup0_Process+0x3a8>
 80059c4:	0086      	lsls	r6, r0, #2
 80059c6:	f106 4e80 	add.w	lr, r6, #1073741824	; 0x40000000
 80059ca:	f50e 4cb8 	add.w	ip, lr, #23552	; 0x5c00
 80059ce:	f8dc 5000 	ldr.w	r5, [ip]
 80059d2:	f405 5640 	and.w	r6, r5, #12288	; 0x3000
 80059d6:	4d1d      	ldr	r5, [pc, #116]	; (8005a4c <Setup0_Process+0x3c4>)
 80059d8:	782d      	ldrb	r5, [r5, #0]
 80059da:	42a8      	cmp	r0, r5
 80059dc:	f4bf ae92 	bcs.w	8005704 <Setup0_Process+0x7c>
 80059e0:	f013 0f70 	tst.w	r3, #112	; 0x70
 80059e4:	f47f ae8e 	bne.w	8005704 <Setup0_Process+0x7c>
 80059e8:	2e00      	cmp	r6, #0
 80059ea:	f43f ae8b 	beq.w	8005704 <Setup0_Process+0x7c>
 80059ee:	4b18      	ldr	r3, [pc, #96]	; (8005a50 <Setup0_Process+0x3c8>)
 80059f0:	e6d3      	b.n	800579a <Setup0_Process+0x112>
 80059f2:	4914      	ldr	r1, [pc, #80]	; (8005a44 <Setup0_Process+0x3bc>)
 80059f4:	680a      	ldr	r2, [r1, #0]
 80059f6:	f892 102c 	ldrb.w	r1, [r2, #44]	; 0x2c
 80059fa:	e767      	b.n	80058cc <Setup0_Process+0x244>
 80059fc:	fb92 fcf1 	sdiv	ip, r2, r1
 8005a00:	fb01 201c 	mls	r0, r1, ip, r2
 8005a04:	2800      	cmp	r0, #0
 8005a06:	f47f af61 	bne.w	80058cc <Setup0_Process+0x244>
 8005a0a:	4a12      	ldr	r2, [pc, #72]	; (8005a54 <Setup0_Process+0x3cc>)
 8005a0c:	2001      	movs	r0, #1
 8005a0e:	7010      	strb	r0, [r2, #0]
 8005a10:	e75c      	b.n	80058cc <Setup0_Process+0x244>
 8005a12:	4d0c      	ldr	r5, [pc, #48]	; (8005a44 <Setup0_Process+0x3bc>)
 8005a14:	7950      	ldrb	r0, [r2, #5]
 8005a16:	682b      	ldr	r3, [r5, #0]
 8005a18:	699a      	ldr	r2, [r3, #24]
 8005a1a:	4790      	blx	r2
 8005a1c:	2800      	cmp	r0, #0
 8005a1e:	f47f af09 	bne.w	8005834 <Setup0_Process+0x1ac>
 8005a22:	6822      	ldr	r2, [r4, #0]
 8005a24:	7a91      	ldrb	r1, [r2, #10]
 8005a26:	2900      	cmp	r1, #0
 8005a28:	f47f af18 	bne.w	800585c <Setup0_Process+0x1d4>
 8005a2c:	7851      	ldrb	r1, [r2, #1]
 8005a2e:	e66a      	b.n	8005706 <Setup0_Process+0x7e>
 8005a30:	0086      	lsls	r6, r0, #2
 8005a32:	f106 4e80 	add.w	lr, r6, #1073741824	; 0x40000000
 8005a36:	f50e 45b8 	add.w	r5, lr, #23552	; 0x5c00
 8005a3a:	682e      	ldr	r6, [r5, #0]
 8005a3c:	f006 0630 	and.w	r6, r6, #48	; 0x30
 8005a40:	e7c9      	b.n	80059d6 <Setup0_Process+0x34e>
 8005a42:	bf00      	nop
 8005a44:	200004e4 	.word	0x200004e4
 8005a48:	20000508 	.word	0x20000508
 8005a4c:	200000cc 	.word	0x200000cc
 8005a50:	08005175 	.word	0x08005175
 8005a54:	20000234 	.word	0x20000234

08005a58 <SetDeviceAddress>:
 8005a58:	b4f0      	push	{r4, r5, r6, r7}
 8005a5a:	4b2a      	ldr	r3, [pc, #168]	; (8005b04 <SetDeviceAddress+0xac>)
 8005a5c:	781e      	ldrb	r6, [r3, #0]
 8005a5e:	2e00      	cmp	r6, #0
 8005a60:	d049      	beq.n	8005af6 <SetDeviceAddress+0x9e>
 8005a62:	4a29      	ldr	r2, [pc, #164]	; (8005b08 <SetDeviceAddress+0xb0>)
 8005a64:	f640 7c0f 	movw	ip, #3855	; 0xf0f
 8005a68:	6814      	ldr	r4, [r2, #0]
 8005a6a:	1e71      	subs	r1, r6, #1
 8005a6c:	ea04 030c 	and.w	r3, r4, ip
 8005a70:	f443 4c00 	orr.w	ip, r3, #32768	; 0x8000
 8005a74:	f04c 0380 	orr.w	r3, ip, #128	; 0x80
 8005a78:	6013      	str	r3, [r2, #0]
 8005a7a:	2301      	movs	r3, #1
 8005a7c:	429e      	cmp	r6, r3
 8005a7e:	f001 0101 	and.w	r1, r1, #1
 8005a82:	d938      	bls.n	8005af6 <SetDeviceAddress+0x9e>
 8005a84:	b169      	cbz	r1, 8005aa2 <SetDeviceAddress+0x4a>
 8005a86:	4921      	ldr	r1, [pc, #132]	; (8005b0c <SetDeviceAddress+0xb4>)
 8005a88:	f640 720f 	movw	r2, #3855	; 0xf0f
 8005a8c:	680c      	ldr	r4, [r1, #0]
 8005a8e:	f248 0c81 	movw	ip, #32897	; 0x8081
 8005a92:	ea04 0302 	and.w	r3, r4, r2
 8005a96:	ea43 020c 	orr.w	r2, r3, ip
 8005a9a:	2302      	movs	r3, #2
 8005a9c:	429e      	cmp	r6, r3
 8005a9e:	600a      	str	r2, [r1, #0]
 8005aa0:	d929      	bls.n	8005af6 <SetDeviceAddress+0x9e>
 8005aa2:	b2dc      	uxtb	r4, r3
 8005aa4:	00a2      	lsls	r2, r4, #2
 8005aa6:	f102 4180 	add.w	r1, r2, #1073741824	; 0x40000000
 8005aaa:	f501 4cb8 	add.w	ip, r1, #23552	; 0x5c00
 8005aae:	f8dc 7000 	ldr.w	r7, [ip]
 8005ab2:	3301      	adds	r3, #1
 8005ab4:	b2d9      	uxtb	r1, r3
 8005ab6:	f444 4500 	orr.w	r5, r4, #32768	; 0x8000
 8005aba:	f640 740f 	movw	r4, #3855	; 0xf0f
 8005abe:	ea07 0404 	and.w	r4, r7, r4
 8005ac2:	f045 0280 	orr.w	r2, r5, #128	; 0x80
 8005ac6:	008f      	lsls	r7, r1, #2
 8005ac8:	ea42 0504 	orr.w	r5, r2, r4
 8005acc:	f107 4480 	add.w	r4, r7, #1073741824	; 0x40000000
 8005ad0:	f8cc 5000 	str.w	r5, [ip]
 8005ad4:	f504 44b8 	add.w	r4, r4, #23552	; 0x5c00
 8005ad8:	6825      	ldr	r5, [r4, #0]
 8005ada:	f441 4100 	orr.w	r1, r1, #32768	; 0x8000
 8005ade:	f640 720f 	movw	r2, #3855	; 0xf0f
 8005ae2:	f041 0c80 	orr.w	ip, r1, #128	; 0x80
 8005ae6:	ea05 0202 	and.w	r2, r5, r2
 8005aea:	3301      	adds	r3, #1
 8005aec:	ea4c 0102 	orr.w	r1, ip, r2
 8005af0:	429e      	cmp	r6, r3
 8005af2:	6021      	str	r1, [r4, #0]
 8005af4:	d8d5      	bhi.n	8005aa2 <SetDeviceAddress+0x4a>
 8005af6:	4b06      	ldr	r3, [pc, #24]	; (8005b10 <SetDeviceAddress+0xb8>)
 8005af8:	f040 0080 	orr.w	r0, r0, #128	; 0x80
 8005afc:	6018      	str	r0, [r3, #0]
 8005afe:	bcf0      	pop	{r4, r5, r6, r7}
 8005b00:	4770      	bx	lr
 8005b02:	bf00      	nop
 8005b04:	200000cc 	.word	0x200000cc
 8005b08:	40005c00 	.word	0x40005c00
 8005b0c:	40005c04 	.word	0x40005c04
 8005b10:	40005c4c 	.word	0x40005c4c

08005b14 <In0_Process>:
 8005b14:	b510      	push	{r4, lr}
 8005b16:	4c14      	ldr	r4, [pc, #80]	; (8005b68 <In0_Process+0x54>)
 8005b18:	6823      	ldr	r3, [r4, #0]
 8005b1a:	7a1a      	ldrb	r2, [r3, #8]
 8005b1c:	2a02      	cmp	r2, #2
 8005b1e:	d012      	beq.n	8005b46 <In0_Process+0x32>
 8005b20:	2a04      	cmp	r2, #4
 8005b22:	d010      	beq.n	8005b46 <In0_Process+0x32>
 8005b24:	2a06      	cmp	r2, #6
 8005b26:	d004      	beq.n	8005b32 <In0_Process+0x1e>
 8005b28:	2208      	movs	r2, #8
 8005b2a:	721a      	strb	r2, [r3, #8]
 8005b2c:	e8bd 4010 	ldmia.w	sp!, {r4, lr}
 8005b30:	e518      	b.n	8005564 <Post0_Process>
 8005b32:	785a      	ldrb	r2, [r3, #1]
 8005b34:	2a05      	cmp	r2, #5
 8005b36:	d00b      	beq.n	8005b50 <In0_Process+0x3c>
 8005b38:	490c      	ldr	r1, [pc, #48]	; (8005b6c <In0_Process+0x58>)
 8005b3a:	680b      	ldr	r3, [r1, #0]
 8005b3c:	6898      	ldr	r0, [r3, #8]
 8005b3e:	4780      	blx	r0
 8005b40:	6823      	ldr	r3, [r4, #0]
 8005b42:	2208      	movs	r2, #8
 8005b44:	e7f1      	b.n	8005b2a <In0_Process+0x16>
 8005b46:	f7ff fb73 	bl	8005230 <DataStageIn>
 8005b4a:	6823      	ldr	r3, [r4, #0]
 8005b4c:	7a1a      	ldrb	r2, [r3, #8]
 8005b4e:	e7ec      	b.n	8005b2a <In0_Process+0x16>
 8005b50:	7818      	ldrb	r0, [r3, #0]
 8005b52:	f010 0f7f 	tst.w	r0, #127	; 0x7f
 8005b56:	d1ef      	bne.n	8005b38 <In0_Process+0x24>
 8005b58:	78d8      	ldrb	r0, [r3, #3]
 8005b5a:	f7ff ff7d 	bl	8005a58 <SetDeviceAddress>
 8005b5e:	4a04      	ldr	r2, [pc, #16]	; (8005b70 <In0_Process+0x5c>)
 8005b60:	6811      	ldr	r1, [r2, #0]
 8005b62:	6a0b      	ldr	r3, [r1, #32]
 8005b64:	4798      	blx	r3
 8005b66:	e7e7      	b.n	8005b38 <In0_Process+0x24>
 8005b68:	2000050c 	.word	0x2000050c
 8005b6c:	200004e4 	.word	0x200004e4
 8005b70:	20000508 	.word	0x20000508

08005b74 <NOP_Process>:
 8005b74:	4770      	bx	lr
 8005b76:	bf00      	nop

08005b78 <USB_Init>:
 8005b78:	b508      	push	{r3, lr}
 8005b7a:	4a07      	ldr	r2, [pc, #28]	; (8005b98 <USB_Init+0x20>)
 8005b7c:	4b07      	ldr	r3, [pc, #28]	; (8005b9c <USB_Init+0x24>)
 8005b7e:	2102      	movs	r1, #2
 8005b80:	4807      	ldr	r0, [pc, #28]	; (8005ba0 <USB_Init+0x28>)
 8005b82:	601a      	str	r2, [r3, #0]
 8005b84:	7211      	strb	r1, [r2, #8]
 8005b86:	4b07      	ldr	r3, [pc, #28]	; (8005ba4 <USB_Init+0x2c>)
 8005b88:	4907      	ldr	r1, [pc, #28]	; (8005ba8 <USB_Init+0x30>)
 8005b8a:	4a08      	ldr	r2, [pc, #32]	; (8005bac <USB_Init+0x34>)
 8005b8c:	6018      	str	r0, [r3, #0]
 8005b8e:	6011      	str	r1, [r2, #0]
 8005b90:	6803      	ldr	r3, [r0, #0]
 8005b92:	4798      	blx	r3
 8005b94:	bd08      	pop	{r3, pc}
 8005b96:	bf00      	nop
 8005b98:	200004ec 	.word	0x200004ec
 8005b9c:	2000050c 	.word	0x2000050c
 8005ba0:	200000d0 	.word	0x200000d0
 8005ba4:	200004e4 	.word	0x200004e4
 8005ba8:	20000100 	.word	0x20000100
 8005bac:	20000508 	.word	0x20000508

08005bb0 <CTR_LP>:
 8005bb0:	e92d 4ff0 	stmdb	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
 8005bb4:	4f67      	ldr	r7, [pc, #412]	; (8005d54 <CTR_LP+0x1a4>)
 8005bb6:	b083      	sub	sp, #12
 8005bb8:	2300      	movs	r3, #0
 8005bba:	f8df a1ac 	ldr.w	sl, [pc, #428]	; 8005d68 <CTR_LP+0x1b8>
 8005bbe:	f8df b1b4 	ldr.w	fp, [pc, #436]	; 8005d74 <CTR_LP+0x1c4>
 8005bc2:	f8df 819c 	ldr.w	r8, [pc, #412]	; 8005d60 <CTR_LP+0x1b0>
 8005bc6:	f8ad 3006 	strh.w	r3, [sp, #6]
 8005bca:	46b9      	mov	r9, r7
 8005bcc:	f8db 1000 	ldr.w	r1, [fp]
 8005bd0:	4c61      	ldr	r4, [pc, #388]	; (8005d58 <CTR_LP+0x1a8>)
 8005bd2:	8039      	strh	r1, [r7, #0]
 8005bd4:	f8b7 c000 	ldrh.w	ip, [r7]
 8005bd8:	4d60      	ldr	r5, [pc, #384]	; (8005d5c <CTR_LP+0x1ac>)
 8005bda:	fa0f f08c 	sxth.w	r0, ip
 8005bde:	2800      	cmp	r0, #0
 8005be0:	f64b 72bf 	movw	r2, #49087	; 0xbfbf
 8005be4:	4e5e      	ldr	r6, [pc, #376]	; (8005d60 <CTR_LP+0x1b0>)
 8005be6:	f280 80a8 	bge.w	8005d3a <CTR_LP+0x18a>
 8005bea:	f8b9 c000 	ldrh.w	ip, [r9]
 8005bee:	f00c 000f 	and.w	r0, ip, #15
 8005bf2:	0083      	lsls	r3, r0, #2
 8005bf4:	f103 4180 	add.w	r1, r3, #1073741824	; 0x40000000
 8005bf8:	f501 41b8 	add.w	r1, r1, #23552	; 0x5c00
 8005bfc:	f88a 0000 	strb.w	r0, [sl]
 8005c00:	2800      	cmp	r0, #0
 8005c02:	d136      	bne.n	8005c72 <CTR_LP+0xc2>
 8005c04:	6821      	ldr	r1, [r4, #0]
 8005c06:	f8a8 1000 	strh.w	r1, [r8]
 8005c0a:	f8b8 0000 	ldrh.w	r0, [r8]
 8005c0e:	f000 0c30 	and.w	ip, r0, #48	; 0x30
 8005c12:	f8a5 c000 	strh.w	ip, [r5]
 8005c16:	f8b8 3000 	ldrh.w	r3, [r8]
 8005c1a:	f403 5140 	and.w	r1, r3, #12288	; 0x3000
 8005c1e:	f8a8 1000 	strh.w	r1, [r8]
 8005c22:	6820      	ldr	r0, [r4, #0]
 8005c24:	ea00 0202 	and.w	r2, r0, r2
 8005c28:	f482 5c00 	eor.w	ip, r2, #8192	; 0x2000
 8005c2c:	f08c 0320 	eor.w	r3, ip, #32
 8005c30:	f443 4100 	orr.w	r1, r3, #32768	; 0x8000
 8005c34:	f041 0080 	orr.w	r0, r1, #128	; 0x80
 8005c38:	6020      	str	r0, [r4, #0]
 8005c3a:	f8b9 2000 	ldrh.w	r2, [r9]
 8005c3e:	f012 0f10 	tst.w	r2, #16
 8005c42:	d046      	beq.n	8005cd2 <CTR_LP+0x122>
 8005c44:	6820      	ldr	r0, [r4, #0]
 8005c46:	f8ad 0006 	strh.w	r0, [sp, #6]
 8005c4a:	f8bd c006 	ldrh.w	ip, [sp, #6]
 8005c4e:	f41c 6f00 	tst.w	ip, #2048	; 0x800
 8005c52:	d175      	bne.n	8005d40 <CTR_LP+0x190>
 8005c54:	f8bd c006 	ldrh.w	ip, [sp, #6]
 8005c58:	fa0f f38c 	sxth.w	r3, ip
 8005c5c:	2b00      	cmp	r3, #0
 8005c5e:	dab5      	bge.n	8005bcc <CTR_LP+0x1c>
 8005c60:	6822      	ldr	r2, [r4, #0]
 8005c62:	f640 718f 	movw	r1, #3983	; 0xf8f
 8005c66:	ea02 0001 	and.w	r0, r2, r1
 8005c6a:	6020      	str	r0, [r4, #0]
 8005c6c:	f7ff fc9e 	bl	80055ac <Out0_Process>
 8005c70:	e037      	b.n	8005ce2 <CTR_LP+0x132>
 8005c72:	680e      	ldr	r6, [r1, #0]
 8005c74:	f8ad 6006 	strh.w	r6, [sp, #6]
 8005c78:	f8bd 5006 	ldrh.w	r5, [sp, #6]
 8005c7c:	b22c      	sxth	r4, r5
 8005c7e:	2c00      	cmp	r4, #0
 8005c80:	db19      	blt.n	8005cb6 <CTR_LP+0x106>
 8005c82:	f8bd c006 	ldrh.w	ip, [sp, #6]
 8005c86:	f01c 0f80 	tst.w	ip, #128	; 0x80
 8005c8a:	d09f      	beq.n	8005bcc <CTR_LP+0x1c>
 8005c8c:	f89a 3000 	ldrb.w	r3, [sl]
 8005c90:	f648 7e0f 	movw	lr, #36623	; 0x8f0f
 8005c94:	0099      	lsls	r1, r3, #2
 8005c96:	f101 4c80 	add.w	ip, r1, #1073741824	; 0x40000000
 8005c9a:	f50c 44b8 	add.w	r4, ip, #23552	; 0x5c00
 8005c9e:	6826      	ldr	r6, [r4, #0]
 8005ca0:	4b30      	ldr	r3, [pc, #192]	; (8005d64 <CTR_LP+0x1b4>)
 8005ca2:	ea06 050e 	and.w	r5, r6, lr
 8005ca6:	6025      	str	r5, [r4, #0]
 8005ca8:	f89a 0000 	ldrb.w	r0, [sl]
 8005cac:	1e42      	subs	r2, r0, #1
 8005cae:	f853 1022 	ldr.w	r1, [r3, r2, lsl #2]
 8005cb2:	4788      	blx	r1
 8005cb4:	e78a      	b.n	8005bcc <CTR_LP+0x1c>
 8005cb6:	680e      	ldr	r6, [r1, #0]
 8005cb8:	f640 7e8f 	movw	lr, #3983	; 0xf8f
 8005cbc:	ea06 050e 	and.w	r5, r6, lr
 8005cc0:	4c29      	ldr	r4, [pc, #164]	; (8005d68 <CTR_LP+0x1b8>)
 8005cc2:	600d      	str	r5, [r1, #0]
 8005cc4:	7820      	ldrb	r0, [r4, #0]
 8005cc6:	4b29      	ldr	r3, [pc, #164]	; (8005d6c <CTR_LP+0x1bc>)
 8005cc8:	1e42      	subs	r2, r0, #1
 8005cca:	f853 1022 	ldr.w	r1, [r3, r2, lsl #2]
 8005cce:	4788      	blx	r1
 8005cd0:	e7d7      	b.n	8005c82 <CTR_LP+0xd2>
 8005cd2:	6823      	ldr	r3, [r4, #0]
 8005cd4:	f648 7e0f 	movw	lr, #36623	; 0x8f0f
 8005cd8:	ea03 020e 	and.w	r2, r3, lr
 8005cdc:	6022      	str	r2, [r4, #0]
 8005cde:	f7ff ff19 	bl	8005b14 <In0_Process>
 8005ce2:	6823      	ldr	r3, [r4, #0]
 8005ce4:	8831      	ldrh	r1, [r6, #0]
 8005ce6:	f64b 7ebf 	movw	lr, #49087	; 0xbfbf
 8005cea:	f401 5c80 	and.w	ip, r1, #4096	; 0x1000
 8005cee:	fa1f f08c 	uxth.w	r0, ip
 8005cf2:	ea03 030e 	and.w	r3, r3, lr
 8005cf6:	b108      	cbz	r0, 8005cfc <CTR_LP+0x14c>
 8005cf8:	f483 5380 	eor.w	r3, r3, #4096	; 0x1000
 8005cfc:	f8b6 e000 	ldrh.w	lr, [r6]
 8005d00:	f40e 5000 	and.w	r0, lr, #8192	; 0x2000
 8005d04:	b282      	uxth	r2, r0
 8005d06:	b10a      	cbz	r2, 8005d0c <CTR_LP+0x15c>
 8005d08:	f483 5300 	eor.w	r3, r3, #8192	; 0x2000
 8005d0c:	882a      	ldrh	r2, [r5, #0]
 8005d0e:	f002 0c10 	and.w	ip, r2, #16
 8005d12:	fa1f f18c 	uxth.w	r1, ip
 8005d16:	b109      	cbz	r1, 8005d1c <CTR_LP+0x16c>
 8005d18:	f083 0310 	eor.w	r3, r3, #16
 8005d1c:	8829      	ldrh	r1, [r5, #0]
 8005d1e:	f001 0e20 	and.w	lr, r1, #32
 8005d22:	fa1f f08e 	uxth.w	r0, lr
 8005d26:	b108      	cbz	r0, 8005d2c <CTR_LP+0x17c>
 8005d28:	f083 0320 	eor.w	r3, r3, #32
 8005d2c:	4810      	ldr	r0, [pc, #64]	; (8005d70 <CTR_LP+0x1c0>)
 8005d2e:	ea43 0c00 	orr.w	ip, r3, r0
 8005d32:	4b09      	ldr	r3, [pc, #36]	; (8005d58 <CTR_LP+0x1a8>)
 8005d34:	fa1f f28c 	uxth.w	r2, ip
 8005d38:	601a      	str	r2, [r3, #0]
 8005d3a:	b003      	add	sp, #12
 8005d3c:	e8bd 8ff0 	ldmia.w	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
 8005d40:	6822      	ldr	r2, [r4, #0]
 8005d42:	f640 7e8f 	movw	lr, #3983	; 0xf8f
 8005d46:	ea02 010e 	and.w	r1, r2, lr
 8005d4a:	6021      	str	r1, [r4, #0]
 8005d4c:	f7ff fc9c 	bl	8005688 <Setup0_Process>
 8005d50:	e7c7      	b.n	8005ce2 <CTR_LP+0x132>
 8005d52:	bf00      	nop
 8005d54:	200004d4 	.word	0x200004d4
 8005d58:	40005c00 	.word	0x40005c00
 8005d5c:	20000514 	.word	0x20000514
 8005d60:	20000512 	.word	0x20000512
 8005d64:	20000094 	.word	0x20000094
 8005d68:	200004e8 	.word	0x200004e8
 8005d6c:	200000b0 	.word	0x200000b0
 8005d70:	ffff8080 	.word	0xffff8080
 8005d74:	40005c44 	.word	0x40005c44

08005d78 <CTR_HP>:
 8005d78:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
 8005d7a:	4c1f      	ldr	r4, [pc, #124]	; (8005df8 <CTR_HP+0x80>)
 8005d7c:	4d1f      	ldr	r5, [pc, #124]	; (8005dfc <CTR_HP+0x84>)
 8005d7e:	4e20      	ldr	r6, [pc, #128]	; (8005e00 <CTR_HP+0x88>)
 8005d80:	4627      	mov	r7, r4
 8005d82:	6828      	ldr	r0, [r5, #0]
 8005d84:	8020      	strh	r0, [r4, #0]
 8005d86:	8821      	ldrh	r1, [r4, #0]
 8005d88:	b20b      	sxth	r3, r1
 8005d8a:	2b00      	cmp	r3, #0
 8005d8c:	da26      	bge.n	8005ddc <CTR_HP+0x64>
 8005d8e:	f647 72ff 	movw	r2, #32767	; 0x7fff
 8005d92:	602a      	str	r2, [r5, #0]
 8005d94:	f8b7 c000 	ldrh.w	ip, [r7]
 8005d98:	f00c 030f 	and.w	r3, ip, #15
 8005d9c:	0099      	lsls	r1, r3, #2
 8005d9e:	f101 4080 	add.w	r0, r1, #1073741824	; 0x40000000
 8005da2:	7033      	strb	r3, [r6, #0]
 8005da4:	f500 43b8 	add.w	r3, r0, #23552	; 0x5c00
 8005da8:	681a      	ldr	r2, [r3, #0]
 8005daa:	b292      	uxth	r2, r2
 8005dac:	f412 4f00 	tst.w	r2, #32768	; 0x8000
 8005db0:	d115      	bne.n	8005dde <CTR_HP+0x66>
 8005db2:	f012 0f80 	tst.w	r2, #128	; 0x80
 8005db6:	d0e4      	beq.n	8005d82 <CTR_HP+0xa>
 8005db8:	681a      	ldr	r2, [r3, #0]
 8005dba:	f648 7c0f 	movw	ip, #36623	; 0x8f0f
 8005dbe:	ea02 000c 	and.w	r0, r2, ip
 8005dc2:	6018      	str	r0, [r3, #0]
 8005dc4:	7831      	ldrb	r1, [r6, #0]
 8005dc6:	480f      	ldr	r0, [pc, #60]	; (8005e04 <CTR_HP+0x8c>)
 8005dc8:	1e4a      	subs	r2, r1, #1
 8005dca:	f850 3022 	ldr.w	r3, [r0, r2, lsl #2]
 8005dce:	4798      	blx	r3
 8005dd0:	6828      	ldr	r0, [r5, #0]
 8005dd2:	8020      	strh	r0, [r4, #0]
 8005dd4:	8821      	ldrh	r1, [r4, #0]
 8005dd6:	b20b      	sxth	r3, r1
 8005dd8:	2b00      	cmp	r3, #0
 8005dda:	dbd8      	blt.n	8005d8e <CTR_HP+0x16>
 8005ddc:	bdf8      	pop	{r3, r4, r5, r6, r7, pc}
 8005dde:	6819      	ldr	r1, [r3, #0]
 8005de0:	f640 7e8f 	movw	lr, #3983	; 0xf8f
 8005de4:	ea01 000e 	and.w	r0, r1, lr
 8005de8:	6018      	str	r0, [r3, #0]
 8005dea:	7832      	ldrb	r2, [r6, #0]
 8005dec:	4b06      	ldr	r3, [pc, #24]	; (8005e08 <CTR_HP+0x90>)
 8005dee:	1e51      	subs	r1, r2, #1
 8005df0:	f853 0021 	ldr.w	r0, [r3, r1, lsl #2]
 8005df4:	4780      	blx	r0
 8005df6:	e7c4      	b.n	8005d82 <CTR_HP+0xa>
 8005df8:	200004d4 	.word	0x200004d4
 8005dfc:	40005c44 	.word	0x40005c44
 8005e00:	200004e8 	.word	0x200004e8
 8005e04:	20000094 	.word	0x20000094
 8005e08:	200000b0 	.word	0x200000b0

08005e0c <UserToPMABufferCopy>:
 8005e0c:	3201      	adds	r2, #1
 8005e0e:	1052      	asrs	r2, r2, #1
 8005e10:	b4f0      	push	{r4, r5, r6, r7}
 8005e12:	d02d      	beq.n	8005e70 <UserToPMABufferCopy+0x64>
 8005e14:	4603      	mov	r3, r0
 8005e16:	7844      	ldrb	r4, [r0, #1]
 8005e18:	f813 cb02 	ldrb.w	ip, [r3], #2
 8005e1c:	f101 5100 	add.w	r1, r1, #536870912	; 0x20000000
 8005e20:	3a01      	subs	r2, #1
 8005e22:	f501 5540 	add.w	r5, r1, #12288	; 0x3000
 8005e26:	ea4c 2004 	orr.w	r0, ip, r4, lsl #8
 8005e2a:	006d      	lsls	r5, r5, #1
 8005e2c:	4611      	mov	r1, r2
 8005e2e:	8028      	strh	r0, [r5, #0]
 8005e30:	f002 0201 	and.w	r2, r2, #1
 8005e34:	2004      	movs	r0, #4
 8005e36:	b1d9      	cbz	r1, 8005e70 <UserToPMABufferCopy+0x64>
 8005e38:	b142      	cbz	r2, 8005e4c <UserToPMABufferCopy+0x40>
 8005e3a:	785c      	ldrb	r4, [r3, #1]
 8005e3c:	f813 cb02 	ldrb.w	ip, [r3], #2
 8005e40:	2008      	movs	r0, #8
 8005e42:	ea4c 2204 	orr.w	r2, ip, r4, lsl #8
 8005e46:	3901      	subs	r1, #1
 8005e48:	80aa      	strh	r2, [r5, #4]
 8005e4a:	d011      	beq.n	8005e70 <UserToPMABufferCopy+0x64>
 8005e4c:	461a      	mov	r2, r3
 8005e4e:	f812 cb02 	ldrb.w	ip, [r2], #2
 8005e52:	785f      	ldrb	r7, [r3, #1]
 8005e54:	1d04      	adds	r4, r0, #4
 8005e56:	ea4c 2607 	orr.w	r6, ip, r7, lsl #8
 8005e5a:	5346      	strh	r6, [r0, r5]
 8005e5c:	f893 c002 	ldrb.w	ip, [r3, #2]
 8005e60:	7856      	ldrb	r6, [r2, #1]
 8005e62:	1c93      	adds	r3, r2, #2
 8005e64:	ea4c 2006 	orr.w	r0, ip, r6, lsl #8
 8005e68:	5360      	strh	r0, [r4, r5]
 8005e6a:	1d20      	adds	r0, r4, #4
 8005e6c:	3902      	subs	r1, #2
 8005e6e:	d1ed      	bne.n	8005e4c <UserToPMABufferCopy+0x40>
 8005e70:	bcf0      	pop	{r4, r5, r6, r7}
 8005e72:	4770      	bx	lr

08005e74 <PMAToUserBufferCopy>:
 8005e74:	3201      	adds	r2, #1
 8005e76:	1052      	asrs	r2, r2, #1
 8005e78:	b430      	push	{r4, r5}
 8005e7a:	d020      	beq.n	8005ebe <PMAToUserBufferCopy+0x4a>
 8005e7c:	f101 5100 	add.w	r1, r1, #536870912	; 0x20000000
 8005e80:	f501 5340 	add.w	r3, r1, #12288	; 0x3000
 8005e84:	005b      	lsls	r3, r3, #1
 8005e86:	3a01      	subs	r2, #1
 8005e88:	f853 4b04 	ldr.w	r4, [r3], #4
 8005e8c:	4611      	mov	r1, r2
 8005e8e:	f820 4b02 	strh.w	r4, [r0], #2
 8005e92:	f002 0201 	and.w	r2, r2, #1
 8005e96:	b191      	cbz	r1, 8005ebe <PMAToUserBufferCopy+0x4a>
 8005e98:	b12a      	cbz	r2, 8005ea6 <PMAToUserBufferCopy+0x32>
 8005e9a:	f853 2b04 	ldr.w	r2, [r3], #4
 8005e9e:	3901      	subs	r1, #1
 8005ea0:	f820 2b02 	strh.w	r2, [r0], #2
 8005ea4:	d00b      	beq.n	8005ebe <PMAToUserBufferCopy+0x4a>
 8005ea6:	461c      	mov	r4, r3
 8005ea8:	f854 5b04 	ldr.w	r5, [r4], #4
 8005eac:	4602      	mov	r2, r0
 8005eae:	f822 5b02 	strh.w	r5, [r2], #2
 8005eb2:	685d      	ldr	r5, [r3, #4]
 8005eb4:	1d23      	adds	r3, r4, #4
 8005eb6:	8045      	strh	r5, [r0, #2]
 8005eb8:	1c90      	adds	r0, r2, #2
 8005eba:	3902      	subs	r1, #2
 8005ebc:	d1f3      	bne.n	8005ea6 <PMAToUserBufferCopy+0x32>
 8005ebe:	bc30      	pop	{r4, r5}
 8005ec0:	4770      	bx	lr
 8005ec2:	bf00      	nop

08005ec4 <SetCNTR>:
 8005ec4:	4b01      	ldr	r3, [pc, #4]	; (8005ecc <SetCNTR+0x8>)
 8005ec6:	6018      	str	r0, [r3, #0]
 8005ec8:	4770      	bx	lr
 8005eca:	bf00      	nop
 8005ecc:	40005c40 	.word	0x40005c40

08005ed0 <GetCNTR>:
 8005ed0:	4b01      	ldr	r3, [pc, #4]	; (8005ed8 <GetCNTR+0x8>)
 8005ed2:	6818      	ldr	r0, [r3, #0]
 8005ed4:	b280      	uxth	r0, r0
 8005ed6:	4770      	bx	lr
 8005ed8:	40005c40 	.word	0x40005c40

08005edc <SetISTR>:
 8005edc:	4b01      	ldr	r3, [pc, #4]	; (8005ee4 <SetISTR+0x8>)
 8005ede:	6018      	str	r0, [r3, #0]
 8005ee0:	4770      	bx	lr
 8005ee2:	bf00      	nop
 8005ee4:	40005c44 	.word	0x40005c44

08005ee8 <GetISTR>:
 8005ee8:	4b01      	ldr	r3, [pc, #4]	; (8005ef0 <GetISTR+0x8>)
 8005eea:	6818      	ldr	r0, [r3, #0]
 8005eec:	b280      	uxth	r0, r0
 8005eee:	4770      	bx	lr
 8005ef0:	40005c44 	.word	0x40005c44

08005ef4 <GetFNR>:
 8005ef4:	4b01      	ldr	r3, [pc, #4]	; (8005efc <GetFNR+0x8>)
 8005ef6:	6818      	ldr	r0, [r3, #0]
 8005ef8:	b280      	uxth	r0, r0
 8005efa:	4770      	bx	lr
 8005efc:	40005c48 	.word	0x40005c48

08005f00 <SetDADDR>:
 8005f00:	4b01      	ldr	r3, [pc, #4]	; (8005f08 <SetDADDR+0x8>)
 8005f02:	6018      	str	r0, [r3, #0]
 8005f04:	4770      	bx	lr
 8005f06:	bf00      	nop
 8005f08:	40005c4c 	.word	0x40005c4c

08005f0c <GetDADDR>:
 8005f0c:	4b01      	ldr	r3, [pc, #4]	; (8005f14 <GetDADDR+0x8>)
 8005f0e:	6818      	ldr	r0, [r3, #0]
 8005f10:	b280      	uxth	r0, r0
 8005f12:	4770      	bx	lr
 8005f14:	40005c4c 	.word	0x40005c4c

08005f18 <SetBTABLE>:
 8005f18:	f64f 71f8 	movw	r1, #65528	; 0xfff8
 8005f1c:	4a02      	ldr	r2, [pc, #8]	; (8005f28 <SetBTABLE+0x10>)
 8005f1e:	ea00 0301 	and.w	r3, r0, r1
 8005f22:	6013      	str	r3, [r2, #0]
 8005f24:	4770      	bx	lr
 8005f26:	bf00      	nop
 8005f28:	40005c50 	.word	0x40005c50

08005f2c <GetBTABLE>:
 8005f2c:	4b01      	ldr	r3, [pc, #4]	; (8005f34 <GetBTABLE+0x8>)
 8005f2e:	6818      	ldr	r0, [r3, #0]
 8005f30:	b280      	uxth	r0, r0
 8005f32:	4770      	bx	lr
 8005f34:	40005c50 	.word	0x40005c50

08005f38 <SetENDPOINT>:
 8005f38:	0083      	lsls	r3, r0, #2
 8005f3a:	f103 4280 	add.w	r2, r3, #1073741824	; 0x40000000
 8005f3e:	f502 40b8 	add.w	r0, r2, #23552	; 0x5c00
 8005f42:	6001      	str	r1, [r0, #0]
 8005f44:	4770      	bx	lr
 8005f46:	bf00      	nop

08005f48 <GetENDPOINT>:
 8005f48:	0082      	lsls	r2, r0, #2
 8005f4a:	f102 4180 	add.w	r1, r2, #1073741824	; 0x40000000
 8005f4e:	f501 43b8 	add.w	r3, r1, #23552	; 0x5c00
 8005f52:	6818      	ldr	r0, [r3, #0]
 8005f54:	b280      	uxth	r0, r0
 8005f56:	4770      	bx	lr

08005f58 <SetEPType>:
 8005f58:	0080      	lsls	r0, r0, #2
 8005f5a:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 8005f5e:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 8005f62:	6813      	ldr	r3, [r2, #0]
 8005f64:	f648 1c8f 	movw	ip, #35215	; 0x898f
 8005f68:	ea03 000c 	and.w	r0, r3, ip
 8005f6c:	ea40 0301 	orr.w	r3, r0, r1
 8005f70:	6013      	str	r3, [r2, #0]
 8005f72:	4770      	bx	lr

08005f74 <GetEPType>:
 8005f74:	0082      	lsls	r2, r0, #2
 8005f76:	f102 4180 	add.w	r1, r2, #1073741824	; 0x40000000
 8005f7a:	f501 43b8 	add.w	r3, r1, #23552	; 0x5c00
 8005f7e:	6818      	ldr	r0, [r3, #0]
 8005f80:	f400 60c0 	and.w	r0, r0, #1536	; 0x600
 8005f84:	4770      	bx	lr
 8005f86:	bf00      	nop

08005f88 <SetEPTxStatus>:
 8005f88:	b410      	push	{r4}
 8005f8a:	0084      	lsls	r4, r0, #2
 8005f8c:	f104 4080 	add.w	r0, r4, #1073741824	; 0x40000000
 8005f90:	f500 40b8 	add.w	r0, r0, #23552	; 0x5c00
 8005f94:	6804      	ldr	r4, [r0, #0]
 8005f96:	f001 0c10 	and.w	ip, r1, #16
 8005f9a:	f648 73bf 	movw	r3, #36799	; 0x8fbf
 8005f9e:	fa1f f28c 	uxth.w	r2, ip
 8005fa2:	ea04 0303 	and.w	r3, r4, r3
 8005fa6:	b10a      	cbz	r2, 8005fac <SetEPTxStatus+0x24>
 8005fa8:	f083 0310 	eor.w	r3, r3, #16
 8005fac:	f001 0220 	and.w	r2, r1, #32
 8005fb0:	b291      	uxth	r1, r2
 8005fb2:	b109      	cbz	r1, 8005fb8 <SetEPTxStatus+0x30>
 8005fb4:	f083 0320 	eor.w	r3, r3, #32
 8005fb8:	f443 4c00 	orr.w	ip, r3, #32768	; 0x8000
 8005fbc:	f04c 0380 	orr.w	r3, ip, #128	; 0x80
 8005fc0:	6003      	str	r3, [r0, #0]
 8005fc2:	bc10      	pop	{r4}
 8005fc4:	4770      	bx	lr
 8005fc6:	bf00      	nop

08005fc8 <SetEPRxStatus>:
 8005fc8:	b410      	push	{r4}
 8005fca:	0084      	lsls	r4, r0, #2
 8005fcc:	f104 4080 	add.w	r0, r4, #1073741824	; 0x40000000
 8005fd0:	f500 40b8 	add.w	r0, r0, #23552	; 0x5c00
 8005fd4:	6804      	ldr	r4, [r0, #0]
 8005fd6:	f401 5c80 	and.w	ip, r1, #4096	; 0x1000
 8005fda:	f64b 738f 	movw	r3, #49039	; 0xbf8f
 8005fde:	fa1f f28c 	uxth.w	r2, ip
 8005fe2:	ea04 0303 	and.w	r3, r4, r3
 8005fe6:	b10a      	cbz	r2, 8005fec <SetEPRxStatus+0x24>
 8005fe8:	f483 5380 	eor.w	r3, r3, #4096	; 0x1000
 8005fec:	f401 5200 	and.w	r2, r1, #8192	; 0x2000
 8005ff0:	b291      	uxth	r1, r2
 8005ff2:	b109      	cbz	r1, 8005ff8 <SetEPRxStatus+0x30>
 8005ff4:	f483 5300 	eor.w	r3, r3, #8192	; 0x2000
 8005ff8:	f443 4c00 	orr.w	ip, r3, #32768	; 0x8000
 8005ffc:	f04c 0380 	orr.w	r3, ip, #128	; 0x80
 8006000:	6003      	str	r3, [r0, #0]
 8006002:	bc10      	pop	{r4}
 8006004:	4770      	bx	lr
 8006006:	bf00      	nop

08006008 <SetDouBleBuffEPStall>:
 8006008:	0080      	lsls	r0, r0, #2
 800600a:	f100 4380 	add.w	r3, r0, #1073741824	; 0x40000000
 800600e:	f503 43b8 	add.w	r3, r3, #23552	; 0x5c00
 8006012:	681a      	ldr	r2, [r3, #0]
 8006014:	2901      	cmp	r1, #1
 8006016:	b292      	uxth	r2, r2
 8006018:	d005      	beq.n	8006026 <SetDouBleBuffEPStall+0x1e>
 800601a:	2902      	cmp	r1, #2
 800601c:	bf04      	itt	eq
 800601e:	f022 0210 	biceq.w	r2, r2, #16
 8006022:	601a      	streq	r2, [r3, #0]
 8006024:	4770      	bx	lr
 8006026:	f422 5180 	bic.w	r1, r2, #4096	; 0x1000
 800602a:	6019      	str	r1, [r3, #0]
 800602c:	4770      	bx	lr
 800602e:	bf00      	nop

08006030 <GetEPTxStatus>:
 8006030:	0082      	lsls	r2, r0, #2
 8006032:	f102 4180 	add.w	r1, r2, #1073741824	; 0x40000000
 8006036:	f501 43b8 	add.w	r3, r1, #23552	; 0x5c00
 800603a:	6818      	ldr	r0, [r3, #0]
 800603c:	f000 0030 	and.w	r0, r0, #48	; 0x30
 8006040:	4770      	bx	lr
 8006042:	bf00      	nop

08006044 <GetEPRxStatus>:
 8006044:	0082      	lsls	r2, r0, #2
 8006046:	f102 4180 	add.w	r1, r2, #1073741824	; 0x40000000
 800604a:	f501 43b8 	add.w	r3, r1, #23552	; 0x5c00
 800604e:	6818      	ldr	r0, [r3, #0]
 8006050:	f400 5040 	and.w	r0, r0, #12288	; 0x3000
 8006054:	4770      	bx	lr
 8006056:	bf00      	nop

08006058 <SetEPTxValid>:
 8006058:	0081      	lsls	r1, r0, #2
 800605a:	f101 4280 	add.w	r2, r1, #1073741824	; 0x40000000
 800605e:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 8006062:	6810      	ldr	r0, [r2, #0]
 8006064:	f648 73bf 	movw	r3, #36799	; 0x8fbf
 8006068:	ea00 0c03 	and.w	ip, r0, r3
 800606c:	f08c 0130 	eor.w	r1, ip, #48	; 0x30
 8006070:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 8006074:	f040 0380 	orr.w	r3, r0, #128	; 0x80
 8006078:	6013      	str	r3, [r2, #0]
 800607a:	4770      	bx	lr

0800607c <SetEPRxValid>:
 800607c:	0081      	lsls	r1, r0, #2
 800607e:	f101 4280 	add.w	r2, r1, #1073741824	; 0x40000000
 8006082:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 8006086:	6810      	ldr	r0, [r2, #0]
 8006088:	f64b 738f 	movw	r3, #49039	; 0xbf8f
 800608c:	ea00 0c03 	and.w	ip, r0, r3
 8006090:	f48c 5140 	eor.w	r1, ip, #12288	; 0x3000
 8006094:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 8006098:	f040 0380 	orr.w	r3, r0, #128	; 0x80
 800609c:	6013      	str	r3, [r2, #0]
 800609e:	4770      	bx	lr

080060a0 <SetEP_KIND>:
 80060a0:	0080      	lsls	r0, r0, #2
 80060a2:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 80060a6:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 80060aa:	6813      	ldr	r3, [r2, #0]
 80060ac:	f640 6c0f 	movw	ip, #3599	; 0xe0f
 80060b0:	ea03 010c 	and.w	r1, r3, ip
 80060b4:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 80060b8:	f440 73c0 	orr.w	r3, r0, #384	; 0x180
 80060bc:	6013      	str	r3, [r2, #0]
 80060be:	4770      	bx	lr

080060c0 <ClearEP_KIND>:
 80060c0:	0080      	lsls	r0, r0, #2
 80060c2:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 80060c6:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 80060ca:	6813      	ldr	r3, [r2, #0]
 80060cc:	f640 6c0f 	movw	ip, #3599	; 0xe0f
 80060d0:	ea03 010c 	and.w	r1, r3, ip
 80060d4:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 80060d8:	f040 0380 	orr.w	r3, r0, #128	; 0x80
 80060dc:	6013      	str	r3, [r2, #0]
 80060de:	4770      	bx	lr

080060e0 <Clear_Status_Out>:
 80060e0:	0080      	lsls	r0, r0, #2
 80060e2:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 80060e6:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 80060ea:	6813      	ldr	r3, [r2, #0]
 80060ec:	f640 6c0f 	movw	ip, #3599	; 0xe0f
 80060f0:	ea03 010c 	and.w	r1, r3, ip
 80060f4:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 80060f8:	f040 0380 	orr.w	r3, r0, #128	; 0x80
 80060fc:	6013      	str	r3, [r2, #0]
 80060fe:	4770      	bx	lr

08006100 <Set_Status_Out>:
 8006100:	0080      	lsls	r0, r0, #2
 8006102:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 8006106:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 800610a:	6813      	ldr	r3, [r2, #0]
 800610c:	f640 6c0f 	movw	ip, #3599	; 0xe0f
 8006110:	ea03 010c 	and.w	r1, r3, ip
 8006114:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 8006118:	f440 73c0 	orr.w	r3, r0, #384	; 0x180
 800611c:	6013      	str	r3, [r2, #0]
 800611e:	4770      	bx	lr

08006120 <SetEPDoubleBuff>:
 8006120:	0080      	lsls	r0, r0, #2
 8006122:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 8006126:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 800612a:	6813      	ldr	r3, [r2, #0]
 800612c:	f640 6c0f 	movw	ip, #3599	; 0xe0f
 8006130:	ea03 010c 	and.w	r1, r3, ip
 8006134:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 8006138:	f440 73c0 	orr.w	r3, r0, #384	; 0x180
 800613c:	6013      	str	r3, [r2, #0]
 800613e:	4770      	bx	lr

08006140 <ClearEPDoubleBuff>:
 8006140:	0080      	lsls	r0, r0, #2
 8006142:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 8006146:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 800614a:	6813      	ldr	r3, [r2, #0]
 800614c:	f640 6c0f 	movw	ip, #3599	; 0xe0f
 8006150:	ea03 010c 	and.w	r1, r3, ip
 8006154:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 8006158:	f040 0380 	orr.w	r3, r0, #128	; 0x80
 800615c:	6013      	str	r3, [r2, #0]
 800615e:	4770      	bx	lr

08006160 <GetTxStallStatus>:
 8006160:	0083      	lsls	r3, r0, #2
 8006162:	f103 4080 	add.w	r0, r3, #1073741824	; 0x40000000
 8006166:	f500 4cb8 	add.w	ip, r0, #23552	; 0x5c00
 800616a:	f8dc 2000 	ldr.w	r2, [ip]
 800616e:	f002 0130 	and.w	r1, r2, #48	; 0x30
 8006172:	f1a1 0310 	sub.w	r3, r1, #16
 8006176:	4258      	negs	r0, r3
 8006178:	eb40 0003 	adc.w	r0, r0, r3
 800617c:	4770      	bx	lr
 800617e:	bf00      	nop

08006180 <GetRxStallStatus>:
 8006180:	0082      	lsls	r2, r0, #2
 8006182:	f102 4080 	add.w	r0, r2, #1073741824	; 0x40000000
 8006186:	f500 4cb8 	add.w	ip, r0, #23552	; 0x5c00
 800618a:	f8dc 3000 	ldr.w	r3, [ip]
 800618e:	f403 5140 	and.w	r1, r3, #12288	; 0x3000
 8006192:	f5a1 5280 	sub.w	r2, r1, #4096	; 0x1000
 8006196:	4250      	negs	r0, r2
 8006198:	eb40 0002 	adc.w	r0, r0, r2
 800619c:	4770      	bx	lr
 800619e:	bf00      	nop

080061a0 <ClearEP_CTR_RX>:
 80061a0:	0082      	lsls	r2, r0, #2
 80061a2:	f102 4c80 	add.w	ip, r2, #1073741824	; 0x40000000
 80061a6:	f50c 42b8 	add.w	r2, ip, #23552	; 0x5c00
 80061aa:	6811      	ldr	r1, [r2, #0]
 80061ac:	f640 708f 	movw	r0, #3983	; 0xf8f
 80061b0:	ea01 0300 	and.w	r3, r1, r0
 80061b4:	6013      	str	r3, [r2, #0]
 80061b6:	4770      	bx	lr

080061b8 <ClearEP_CTR_TX>:
 80061b8:	0082      	lsls	r2, r0, #2
 80061ba:	f102 4c80 	add.w	ip, r2, #1073741824	; 0x40000000
 80061be:	f50c 42b8 	add.w	r2, ip, #23552	; 0x5c00
 80061c2:	6811      	ldr	r1, [r2, #0]
 80061c4:	f648 700f 	movw	r0, #36623	; 0x8f0f
 80061c8:	ea01 0300 	and.w	r3, r1, r0
 80061cc:	6013      	str	r3, [r2, #0]
 80061ce:	4770      	bx	lr

080061d0 <ToggleDTOG_RX>:
 80061d0:	0080      	lsls	r0, r0, #2
 80061d2:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 80061d6:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 80061da:	6813      	ldr	r3, [r2, #0]
 80061dc:	f640 7c0f 	movw	ip, #3855	; 0xf0f
 80061e0:	ea03 010c 	and.w	r1, r3, ip
 80061e4:	f441 4040 	orr.w	r0, r1, #49152	; 0xc000
 80061e8:	f040 0380 	orr.w	r3, r0, #128	; 0x80
 80061ec:	6013      	str	r3, [r2, #0]
 80061ee:	4770      	bx	lr

080061f0 <ToggleDTOG_TX>:
 80061f0:	0080      	lsls	r0, r0, #2
 80061f2:	f100 4280 	add.w	r2, r0, #1073741824	; 0x40000000
 80061f6:	f502 42b8 	add.w	r2, r2, #23552	; 0x5c00
 80061fa:	6813      	ldr	r3, [r2, #0]
 80061fc:	f640 7c0f 	movw	ip, #3855	; 0xf0f
 8006200:	ea03 010c 	and.w	r1, r3, ip
 8006204:	f441 4000 	orr.w	r0, r1, #32768	; 0x8000
 8006208:	f040 03c0 	orr.w	r3, r0, #192	; 0xc0
 800620c:	6013      	str	r3, [r2, #0]
 800620e:	4770      	bx	lr

08006210 <ClearDTOG_RX>:
 8006210:	0080      	lsls	r0, r0, #2
 8006212:	f100 4380 	add.w	r3, r0, #1073741824	; 0x40000000
 8006216:	f503 43b8 	add.w	r3, r3, #23552	; 0x5c00
 800621a:	681a      	ldr	r2, [r3, #0]
 800621c:	f412 4f80 	tst.w	r2, #16384	; 0x4000
 8006220:	d009      	beq.n	8006236 <ClearDTOG_RX+0x26>
 8006222:	6819      	ldr	r1, [r3, #0]
 8006224:	f640 700f 	movw	r0, #3855	; 0xf0f
 8006228:	ea01 0200 	and.w	r2, r1, r0
 800622c:	f442 4c40 	orr.w	ip, r2, #49152	; 0xc000
 8006230:	f04c 0180 	orr.w	r1, ip, #128	; 0x80
 8006234:	6019      	str	r1, [r3, #0]
 8006236:	4770      	bx	lr

08006238 <ClearDTOG_TX>:
 8006238:	0080      	lsls	r0, r0, #2
 800623a:	f100 4380 	add.w	r3, r0, #1073741824	; 0x40000000
 800623e:	f503 43b8 	add.w	r3, r3, #23552	; 0x5c00
 8006242:	681a      	ldr	r2, [r3, #0]
 8006244:	f012 0f40 	tst.w	r2, #64	; 0x40
 8006248:	d009      	beq.n	800625e <ClearDTOG_TX+0x26>
 800624a:	6819      	ldr	r1, [r3, #0]
 800624c:	f640 700f 	movw	r0, #3855	; 0xf0f
 8006250:	ea01 0200 	and.w	r2, r1, r0
 8006254:	f442 4c00 	orr.w	ip, r2, #32768	; 0x8000
 8006258:	f04c 01c0 	orr.w	r1, ip, #192	; 0xc0
 800625c:	6019      	str	r1, [r3, #0]
 800625e:	4770      	bx	lr

08006260 <SetEPAddress>:
 8006260:	0080      	lsls	r0, r0, #2
 8006262:	f100 4380 	add.w	r3, r0, #1073741824	; 0x40000000
 8006266:	f503 43b8 	add.w	r3, r3, #23552	; 0x5c00
 800626a:	6818      	ldr	r0, [r3, #0]
 800626c:	f441 4200 	orr.w	r2, r1, #32768	; 0x8000
 8006270:	f640 7c0f 	movw	ip, #3855	; 0xf0f
 8006274:	f042 0180 	orr.w	r1, r2, #128	; 0x80
 8006278:	ea00 020c 	and.w	r2, r0, ip
 800627c:	ea41 0002 	orr.w	r0, r1, r2
 8006280:	6018      	str	r0, [r3, #0]
 8006282:	4770      	bx	lr

08006284 <GetEPAddress>:
 8006284:	0082      	lsls	r2, r0, #2
 8006286:	f102 4180 	add.w	r1, r2, #1073741824	; 0x40000000
 800628a:	f501 43b8 	add.w	r3, r1, #23552	; 0x5c00
 800628e:	6818      	ldr	r0, [r3, #0]
 8006290:	f000 000f 	and.w	r0, r0, #15
 8006294:	4770      	bx	lr
 8006296:	bf00      	nop

08006298 <SetEPTxAddr>:
 8006298:	4a07      	ldr	r2, [pc, #28]	; (80062b8 <SetEPTxAddr+0x20>)
 800629a:	f64f 73fe 	movw	r3, #65534	; 0xfffe
 800629e:	6812      	ldr	r2, [r2, #0]
 80062a0:	ea01 0303 	and.w	r3, r1, r3
 80062a4:	b292      	uxth	r2, r2
 80062a6:	eb02 0cc0 	add.w	ip, r2, r0, lsl #3
 80062aa:	f10c 5100 	add.w	r1, ip, #536870912	; 0x20000000
 80062ae:	f501 5040 	add.w	r0, r1, #12288	; 0x3000
 80062b2:	0042      	lsls	r2, r0, #1
 80062b4:	6013      	str	r3, [r2, #0]
 80062b6:	4770      	bx	lr
 80062b8:	40005c50 	.word	0x40005c50

080062bc <SetEPRxAddr>:
 80062bc:	b410      	push	{r4}
 80062be:	4b07      	ldr	r3, [pc, #28]	; (80062dc <SetEPRxAddr+0x20>)
 80062c0:	4a07      	ldr	r2, [pc, #28]	; (80062e0 <SetEPRxAddr+0x24>)
 80062c2:	681c      	ldr	r4, [r3, #0]
 80062c4:	f64f 7cfe 	movw	ip, #65534	; 0xfffe
 80062c8:	b2a4      	uxth	r4, r4
 80062ca:	eb04 03c0 	add.w	r3, r4, r0, lsl #3
 80062ce:	1898      	adds	r0, r3, r2
 80062d0:	0042      	lsls	r2, r0, #1
 80062d2:	ea01 030c 	and.w	r3, r1, ip
 80062d6:	6013      	str	r3, [r2, #0]
 80062d8:	bc10      	pop	{r4}
 80062da:	4770      	bx	lr
 80062dc:	40005c50 	.word	0x40005c50
 80062e0:	20003004 	.word	0x20003004

080062e4 <GetEPTxAddr>:
 80062e4:	4906      	ldr	r1, [pc, #24]	; (8006300 <GetEPTxAddr+0x1c>)
 80062e6:	680b      	ldr	r3, [r1, #0]
 80062e8:	fa1f fc83 	uxth.w	ip, r3
 80062ec:	eb0c 02c0 	add.w	r2, ip, r0, lsl #3
 80062f0:	f102 5100 	add.w	r1, r2, #536870912	; 0x20000000
 80062f4:	f501 5040 	add.w	r0, r1, #12288	; 0x3000
 80062f8:	0043      	lsls	r3, r0, #1
 80062fa:	8818      	ldrh	r0, [r3, #0]
 80062fc:	4770      	bx	lr
 80062fe:	bf00      	nop
 8006300:	40005c50 	.word	0x40005c50

08006304 <GetEPRxAddr>:
 8006304:	4a05      	ldr	r2, [pc, #20]	; (800631c <GetEPRxAddr+0x18>)
 8006306:	4906      	ldr	r1, [pc, #24]	; (8006320 <GetEPRxAddr+0x1c>)
 8006308:	6813      	ldr	r3, [r2, #0]
 800630a:	fa1f fc83 	uxth.w	ip, r3
 800630e:	eb0c 02c0 	add.w	r2, ip, r0, lsl #3
 8006312:	1850      	adds	r0, r2, r1
 8006314:	0043      	lsls	r3, r0, #1
 8006316:	8818      	ldrh	r0, [r3, #0]
 8006318:	4770      	bx	lr
 800631a:	bf00      	nop
 800631c:	40005c50 	.word	0x40005c50
 8006320:	20003004 	.word	0x20003004

08006324 <SetEPTxCount>:
 8006324:	4a05      	ldr	r2, [pc, #20]	; (800633c <SetEPTxCount+0x18>)
 8006326:	4b06      	ldr	r3, [pc, #24]	; (8006340 <SetEPTxCount+0x1c>)
 8006328:	6812      	ldr	r2, [r2, #0]
 800632a:	fa1f fc82 	uxth.w	ip, r2
 800632e:	eb0c 02c0 	add.w	r2, ip, r0, lsl #3
 8006332:	18d0      	adds	r0, r2, r3
 8006334:	0043      	lsls	r3, r0, #1
 8006336:	6019      	str	r1, [r3, #0]
 8006338:	4770      	bx	lr
 800633a:	bf00      	nop
 800633c:	40005c50 	.word	0x40005c50
 8006340:	20003002 	.word	0x20003002

08006344 <SetEPCountRxReg>:
 8006344:	293e      	cmp	r1, #62	; 0x3e
 8006346:	d90a      	bls.n	800635e <SetEPCountRxReg+0x1a>
 8006348:	094b      	lsrs	r3, r1, #5
 800634a:	f011 0f1f 	tst.w	r1, #31
 800634e:	d101      	bne.n	8006354 <SetEPCountRxReg+0x10>
 8006350:	3b01      	subs	r3, #1
 8006352:	b29b      	uxth	r3, r3
 8006354:	029a      	lsls	r2, r3, #10
 8006356:	f442 4100 	orr.w	r1, r2, #32768	; 0x8000
 800635a:	6001      	str	r1, [r0, #0]
 800635c:	4770      	bx	lr
 800635e:	084a      	lsrs	r2, r1, #1
 8006360:	f011 0f01 	tst.w	r1, #1
 8006364:	bf18      	it	ne
 8006366:	3201      	addne	r2, #1
 8006368:	0293      	lsls	r3, r2, #10
 800636a:	6003      	str	r3, [r0, #0]
 800636c:	4770      	bx	lr
 800636e:	bf00      	nop

08006370 <SetEPRxCount>:
 8006370:	4a0f      	ldr	r2, [pc, #60]	; (80063b0 <SetEPRxCount+0x40>)
 8006372:	4b10      	ldr	r3, [pc, #64]	; (80063b4 <SetEPRxCount+0x44>)
 8006374:	6812      	ldr	r2, [r2, #0]
 8006376:	fa1f fc82 	uxth.w	ip, r2
 800637a:	eb0c 0203 	add.w	r2, ip, r3
 800637e:	eb02 03c0 	add.w	r3, r2, r0, lsl #3
 8006382:	005b      	lsls	r3, r3, #1
 8006384:	293e      	cmp	r1, #62	; 0x3e
 8006386:	d90a      	bls.n	800639e <SetEPRxCount+0x2e>
 8006388:	094a      	lsrs	r2, r1, #5
 800638a:	f011 0f1f 	tst.w	r1, #31
 800638e:	d101      	bne.n	8006394 <SetEPRxCount+0x24>
 8006390:	1e50      	subs	r0, r2, #1
 8006392:	b282      	uxth	r2, r0
 8006394:	0292      	lsls	r2, r2, #10
 8006396:	f442 4100 	orr.w	r1, r2, #32768	; 0x8000
 800639a:	6019      	str	r1, [r3, #0]
 800639c:	4770      	bx	lr
 800639e:	084a      	lsrs	r2, r1, #1
 80063a0:	f011 0f01 	tst.w	r1, #1
 80063a4:	bf18      	it	ne
 80063a6:	3201      	addne	r2, #1
 80063a8:	0290      	lsls	r0, r2, #10
 80063aa:	6018      	str	r0, [r3, #0]
 80063ac:	4770      	bx	lr
 80063ae:	bf00      	nop
 80063b0:	40005c50 	.word	0x40005c50
 80063b4:	20003006 	.word	0x20003006

080063b8 <GetEPTxCount>:
 80063b8:	4b06      	ldr	r3, [pc, #24]	; (80063d4 <GetEPTxCount+0x1c>)
 80063ba:	4907      	ldr	r1, [pc, #28]	; (80063d8 <GetEPTxCount+0x20>)
 80063bc:	681a      	ldr	r2, [r3, #0]
 80063be:	b293      	uxth	r3, r2
 80063c0:	eb03 0cc0 	add.w	ip, r3, r0, lsl #3
 80063c4:	eb0c 0201 	add.w	r2, ip, r1
 80063c8:	0053      	lsls	r3, r2, #1
 80063ca:	6819      	ldr	r1, [r3, #0]
 80063cc:	0588      	lsls	r0, r1, #22
 80063ce:	0d80      	lsrs	r0, r0, #22
 80063d0:	4770      	bx	lr
 80063d2:	bf00      	nop
 80063d4:	40005c50 	.word	0x40005c50
 80063d8:	20003002 	.word	0x20003002

080063dc <GetEPRxCount>:
 80063dc:	4b06      	ldr	r3, [pc, #24]	; (80063f8 <GetEPRxCount+0x1c>)
 80063de:	4907      	ldr	r1, [pc, #28]	; (80063fc <GetEPRxCount+0x20>)
 80063e0:	681a      	ldr	r2, [r3, #0]
 80063e2:	b293      	uxth	r3, r2
 80063e4:	eb03 0cc0 	add.w	ip, r3, r0, lsl #3
 80063e8:	eb0c 0201 	add.w	r2, ip, r1
 80063ec:	0053      	lsls	r3, r2, #1
 80063ee:	6819      	ldr	r1, [r3, #0]
 80063f0:	0588      	lsls	r0, r1, #22
 80063f2:	0d80      	lsrs	r0, r0, #22
 80063f4:	4770      	bx	lr
 80063f6:	bf00      	nop
 80063f8:	40005c50 	.word	0x40005c50
 80063fc:	20003006 	.word	0x20003006

08006400 <SetEPDblBuffAddr>:
 8006400:	b430      	push	{r4, r5}
 8006402:	4b0f      	ldr	r3, [pc, #60]	; (8006440 <SetEPDblBuffAddr+0x40>)
 8006404:	00c0      	lsls	r0, r0, #3
 8006406:	681d      	ldr	r5, [r3, #0]
 8006408:	f64f 74fe 	movw	r4, #65534	; 0xfffe
 800640c:	fa1f fc85 	uxth.w	ip, r5
 8006410:	eb00 050c 	add.w	r5, r0, ip
 8006414:	f105 5c00 	add.w	ip, r5, #536870912	; 0x20000000
 8006418:	f50c 5540 	add.w	r5, ip, #12288	; 0x3000
 800641c:	ea01 0404 	and.w	r4, r1, r4
 8006420:	006d      	lsls	r5, r5, #1
 8006422:	602c      	str	r4, [r5, #0]
 8006424:	681b      	ldr	r3, [r3, #0]
 8006426:	4907      	ldr	r1, [pc, #28]	; (8006444 <SetEPDblBuffAddr+0x44>)
 8006428:	b29b      	uxth	r3, r3
 800642a:	eb00 0c03 	add.w	ip, r0, r3
 800642e:	4461      	add	r1, ip
 8006430:	f64f 70fe 	movw	r0, #65534	; 0xfffe
 8006434:	0049      	lsls	r1, r1, #1
 8006436:	ea02 0300 	and.w	r3, r2, r0
 800643a:	600b      	str	r3, [r1, #0]
 800643c:	bc30      	pop	{r4, r5}
 800643e:	4770      	bx	lr
 8006440:	40005c50 	.word	0x40005c50
 8006444:	20003004 	.word	0x20003004

08006448 <SetEPDblBuf0Addr>:
 8006448:	4a07      	ldr	r2, [pc, #28]	; (8006468 <SetEPDblBuf0Addr+0x20>)
 800644a:	f64f 73fe 	movw	r3, #65534	; 0xfffe
 800644e:	6812      	ldr	r2, [r2, #0]
 8006450:	ea01 0303 	and.w	r3, r1, r3
 8006454:	b292      	uxth	r2, r2
 8006456:	eb02 0cc0 	add.w	ip, r2, r0, lsl #3
 800645a:	f10c 5100 	add.w	r1, ip, #536870912	; 0x20000000
 800645e:	f501 5040 	add.w	r0, r1, #12288	; 0x3000
 8006462:	0042      	lsls	r2, r0, #1
 8006464:	6013      	str	r3, [r2, #0]
 8006466:	4770      	bx	lr
 8006468:	40005c50 	.word	0x40005c50

0800646c <SetEPDblBuf1Addr>:
 800646c:	b410      	push	{r4}
 800646e:	4b07      	ldr	r3, [pc, #28]	; (800648c <SetEPDblBuf1Addr+0x20>)
 8006470:	4a07      	ldr	r2, [pc, #28]	; (8006490 <SetEPDblBuf1Addr+0x24>)
 8006472:	681c      	ldr	r4, [r3, #0]
 8006474:	f64f 7cfe 	movw	ip, #65534	; 0xfffe
 8006478:	b2a4      	uxth	r4, r4
 800647a:	eb04 03c0 	add.w	r3, r4, r0, lsl #3
 800647e:	1898      	adds	r0, r3, r2
 8006480:	0042      	lsls	r2, r0, #1
 8006482:	ea01 030c 	and.w	r3, r1, ip
 8006486:	6013      	str	r3, [r2, #0]
 8006488:	bc10      	pop	{r4}
 800648a:	4770      	bx	lr
 800648c:	40005c50 	.word	0x40005c50
 8006490:	20003004 	.word	0x20003004

08006494 <GetEPDblBuf0Addr>:
 8006494:	4906      	ldr	r1, [pc, #24]	; (80064b0 <GetEPDblBuf0Addr+0x1c>)
 8006496:	680b      	ldr	r3, [r1, #0]
 8006498:	fa1f fc83 	uxth.w	ip, r3
 800649c:	eb0c 02c0 	add.w	r2, ip, r0, lsl #3
 80064a0:	f102 5100 	add.w	r1, r2, #536870912	; 0x20000000
 80064a4:	f501 5040 	add.w	r0, r1, #12288	; 0x3000
 80064a8:	0043      	lsls	r3, r0, #1
 80064aa:	8818      	ldrh	r0, [r3, #0]
 80064ac:	4770      	bx	lr
 80064ae:	bf00      	nop
 80064b0:	40005c50 	.word	0x40005c50

080064b4 <GetEPDblBuf1Addr>:
 80064b4:	4a05      	ldr	r2, [pc, #20]	; (80064cc <GetEPDblBuf1Addr+0x18>)
 80064b6:	4906      	ldr	r1, [pc, #24]	; (80064d0 <GetEPDblBuf1Addr+0x1c>)
 80064b8:	6813      	ldr	r3, [r2, #0]
 80064ba:	fa1f fc83 	uxth.w	ip, r3
 80064be:	eb0c 02c0 	add.w	r2, ip, r0, lsl #3
 80064c2:	1850      	adds	r0, r2, r1
 80064c4:	0043      	lsls	r3, r0, #1
 80064c6:	8818      	ldrh	r0, [r3, #0]
 80064c8:	4770      	bx	lr
 80064ca:	bf00      	nop
 80064cc:	40005c50 	.word	0x40005c50
 80064d0:	20003004 	.word	0x20003004

080064d4 <SetEPDblBuffCount>:
 80064d4:	2901      	cmp	r1, #1
 80064d6:	b410      	push	{r4}
 80064d8:	d003      	beq.n	80064e2 <SetEPDblBuffCount+0xe>
 80064da:	2902      	cmp	r1, #2
 80064dc:	d024      	beq.n	8006528 <SetEPDblBuffCount+0x54>
 80064de:	bc10      	pop	{r4}
 80064e0:	4770      	bx	lr
 80064e2:	4924      	ldr	r1, [pc, #144]	; (8006574 <SetEPDblBuffCount+0xa0>)
 80064e4:	4b24      	ldr	r3, [pc, #144]	; (8006578 <SetEPDblBuffCount+0xa4>)
 80064e6:	680c      	ldr	r4, [r1, #0]
 80064e8:	00c0      	lsls	r0, r0, #3
 80064ea:	fa1f fc84 	uxth.w	ip, r4
 80064ee:	eb0c 0403 	add.w	r4, ip, r3
 80064f2:	1823      	adds	r3, r4, r0
 80064f4:	005b      	lsls	r3, r3, #1
 80064f6:	2a3e      	cmp	r2, #62	; 0x3e
 80064f8:	d929      	bls.n	800654e <SetEPDblBuffCount+0x7a>
 80064fa:	0954      	lsrs	r4, r2, #5
 80064fc:	f012 0f1f 	tst.w	r2, #31
 8006500:	d101      	bne.n	8006506 <SetEPDblBuffCount+0x32>
 8006502:	1e62      	subs	r2, r4, #1
 8006504:	b294      	uxth	r4, r2
 8006506:	02a2      	lsls	r2, r4, #10
 8006508:	f442 4200 	orr.w	r2, r2, #32768	; 0x8000
 800650c:	601a      	str	r2, [r3, #0]
 800650e:	680a      	ldr	r2, [r1, #0]
 8006510:	4b1a      	ldr	r3, [pc, #104]	; (800657c <SetEPDblBuffCount+0xa8>)
 8006512:	fa1f fc82 	uxth.w	ip, r2
 8006516:	eb0c 0103 	add.w	r1, ip, r3
 800651a:	1808      	adds	r0, r1, r0
 800651c:	02a2      	lsls	r2, r4, #10
 800651e:	0040      	lsls	r0, r0, #1
 8006520:	f442 4400 	orr.w	r4, r2, #32768	; 0x8000
 8006524:	6004      	str	r4, [r0, #0]
 8006526:	e7da      	b.n	80064de <SetEPDblBuffCount+0xa>
 8006528:	4912      	ldr	r1, [pc, #72]	; (8006574 <SetEPDblBuffCount+0xa0>)
 800652a:	00c0      	lsls	r0, r0, #3
 800652c:	680c      	ldr	r4, [r1, #0]
 800652e:	4b12      	ldr	r3, [pc, #72]	; (8006578 <SetEPDblBuffCount+0xa4>)
 8006530:	b2a4      	uxth	r4, r4
 8006532:	1904      	adds	r4, r0, r4
 8006534:	18e3      	adds	r3, r4, r3
 8006536:	005b      	lsls	r3, r3, #1
 8006538:	601a      	str	r2, [r3, #0]
 800653a:	6809      	ldr	r1, [r1, #0]
 800653c:	4b0f      	ldr	r3, [pc, #60]	; (800657c <SetEPDblBuffCount+0xa8>)
 800653e:	b289      	uxth	r1, r1
 8006540:	eb00 0c01 	add.w	ip, r0, r1
 8006544:	eb0c 0103 	add.w	r1, ip, r3
 8006548:	0048      	lsls	r0, r1, #1
 800654a:	6002      	str	r2, [r0, #0]
 800654c:	e7c7      	b.n	80064de <SetEPDblBuffCount+0xa>
 800654e:	0854      	lsrs	r4, r2, #1
 8006550:	f012 0f01 	tst.w	r2, #1
 8006554:	bf18      	it	ne
 8006556:	3401      	addne	r4, #1
 8006558:	02a2      	lsls	r2, r4, #10
 800655a:	601a      	str	r2, [r3, #0]
 800655c:	680a      	ldr	r2, [r1, #0]
 800655e:	4b07      	ldr	r3, [pc, #28]	; (800657c <SetEPDblBuffCount+0xa8>)
 8006560:	fa1f fc82 	uxth.w	ip, r2
 8006564:	eb0c 0103 	add.w	r1, ip, r3
 8006568:	1808      	adds	r0, r1, r0
 800656a:	0040      	lsls	r0, r0, #1
 800656c:	02a4      	lsls	r4, r4, #10
 800656e:	6004      	str	r4, [r0, #0]
 8006570:	e7b5      	b.n	80064de <SetEPDblBuffCount+0xa>
 8006572:	bf00      	nop
 8006574:	40005c50 	.word	0x40005c50
 8006578:	20003002 	.word	0x20003002
 800657c:	20003006 	.word	0x20003006

08006580 <SetEPDblBuf0Count>:
 8006580:	2901      	cmp	r1, #1
 8006582:	d00d      	beq.n	80065a0 <SetEPDblBuf0Count+0x20>
 8006584:	2902      	cmp	r1, #2
 8006586:	d000      	beq.n	800658a <SetEPDblBuf0Count+0xa>
 8006588:	4770      	bx	lr
 800658a:	4915      	ldr	r1, [pc, #84]	; (80065e0 <SetEPDblBuf0Count+0x60>)
 800658c:	4b15      	ldr	r3, [pc, #84]	; (80065e4 <SetEPDblBuf0Count+0x64>)
 800658e:	6809      	ldr	r1, [r1, #0]
 8006590:	b289      	uxth	r1, r1
 8006592:	eb01 0cc0 	add.w	ip, r1, r0, lsl #3
 8006596:	eb0c 0003 	add.w	r0, ip, r3
 800659a:	0043      	lsls	r3, r0, #1
 800659c:	601a      	str	r2, [r3, #0]
 800659e:	4770      	bx	lr
 80065a0:	490f      	ldr	r1, [pc, #60]	; (80065e0 <SetEPDblBuf0Count+0x60>)
 80065a2:	4b10      	ldr	r3, [pc, #64]	; (80065e4 <SetEPDblBuf0Count+0x64>)
 80065a4:	6809      	ldr	r1, [r1, #0]
 80065a6:	fa1f fc81 	uxth.w	ip, r1
 80065aa:	eb0c 0103 	add.w	r1, ip, r3
 80065ae:	eb01 03c0 	add.w	r3, r1, r0, lsl #3
 80065b2:	005b      	lsls	r3, r3, #1
 80065b4:	2a3e      	cmp	r2, #62	; 0x3e
 80065b6:	d90a      	bls.n	80065ce <SetEPDblBuf0Count+0x4e>
 80065b8:	0951      	lsrs	r1, r2, #5
 80065ba:	f012 0f1f 	tst.w	r2, #31
 80065be:	d101      	bne.n	80065c4 <SetEPDblBuf0Count+0x44>
 80065c0:	1e48      	subs	r0, r1, #1
 80065c2:	b281      	uxth	r1, r0
 80065c4:	0289      	lsls	r1, r1, #10
 80065c6:	f441 4200 	orr.w	r2, r1, #32768	; 0x8000
 80065ca:	601a      	str	r2, [r3, #0]
 80065cc:	4770      	bx	lr
 80065ce:	0851      	lsrs	r1, r2, #1
 80065d0:	f012 0f01 	tst.w	r2, #1
 80065d4:	bf18      	it	ne
 80065d6:	3101      	addne	r1, #1
 80065d8:	0288      	lsls	r0, r1, #10
 80065da:	6018      	str	r0, [r3, #0]
 80065dc:	4770      	bx	lr
 80065de:	bf00      	nop
 80065e0:	40005c50 	.word	0x40005c50
 80065e4:	20003002 	.word	0x20003002

080065e8 <SetEPDblBuf1Count>:
 80065e8:	2901      	cmp	r1, #1
 80065ea:	d00d      	beq.n	8006608 <SetEPDblBuf1Count+0x20>
 80065ec:	2902      	cmp	r1, #2
 80065ee:	d000      	beq.n	80065f2 <SetEPDblBuf1Count+0xa>
 80065f0:	4770      	bx	lr
 80065f2:	4915      	ldr	r1, [pc, #84]	; (8006648 <SetEPDblBuf1Count+0x60>)
 80065f4:	4b15      	ldr	r3, [pc, #84]	; (800664c <SetEPDblBuf1Count+0x64>)
 80065f6:	6809      	ldr	r1, [r1, #0]
 80065f8:	b289      	uxth	r1, r1
 80065fa:	eb01 0cc0 	add.w	ip, r1, r0, lsl #3
 80065fe:	eb0c 0003 	add.w	r0, ip, r3
 8006602:	0043      	lsls	r3, r0, #1
 8006604:	601a      	str	r2, [r3, #0]
 8006606:	4770      	bx	lr
 8006608:	490f      	ldr	r1, [pc, #60]	; (8006648 <SetEPDblBuf1Count+0x60>)
 800660a:	4b10      	ldr	r3, [pc, #64]	; (800664c <SetEPDblBuf1Count+0x64>)
 800660c:	6809      	ldr	r1, [r1, #0]
 800660e:	fa1f fc81 	uxth.w	ip, r1
 8006612:	eb0c 0103 	add.w	r1, ip, r3
 8006616:	eb01 03c0 	add.w	r3, r1, r0, lsl #3
 800661a:	005b      	lsls	r3, r3, #1
 800661c:	2a3e      	cmp	r2, #62	; 0x3e
 800661e:	d90a      	bls.n	8006636 <SetEPDblBuf1Count+0x4e>
 8006620:	0951      	lsrs	r1, r2, #5
 8006622:	f012 0f1f 	tst.w	r2, #31
 8006626:	d101      	bne.n	800662c <SetEPDblBuf1Count+0x44>
 8006628:	1e48      	subs	r0, r1, #1
 800662a:	b281      	uxth	r1, r0
 800662c:	0289      	lsls	r1, r1, #10
 800662e:	f441 4200 	orr.w	r2, r1, #32768	; 0x8000
 8006632:	601a      	str	r2, [r3, #0]
 8006634:	4770      	bx	lr
 8006636:	0851      	lsrs	r1, r2, #1
 8006638:	f012 0f01 	tst.w	r2, #1
 800663c:	bf18      	it	ne
 800663e:	3101      	addne	r1, #1
 8006640:	0288      	lsls	r0, r1, #10
 8006642:	6018      	str	r0, [r3, #0]
 8006644:	4770      	bx	lr
 8006646:	bf00      	nop
 8006648:	40005c50 	.word	0x40005c50
 800664c:	20003006 	.word	0x20003006

08006650 <GetEPDblBuf0Count>:
 8006650:	4b06      	ldr	r3, [pc, #24]	; (800666c <GetEPDblBuf0Count+0x1c>)
 8006652:	4907      	ldr	r1, [pc, #28]	; (8006670 <GetEPDblBuf0Count+0x20>)
 8006654:	681a      	ldr	r2, [r3, #0]
 8006656:	b293      	uxth	r3, r2
 8006658:	eb03 0cc0 	add.w	ip, r3, r0, lsl #3
 800665c:	eb0c 0201 	add.w	r2, ip, r1
 8006660:	0053      	lsls	r3, r2, #1
 8006662:	6819      	ldr	r1, [r3, #0]
 8006664:	0588      	lsls	r0, r1, #22
 8006666:	0d80      	lsrs	r0, r0, #22
 8006668:	4770      	bx	lr
 800666a:	bf00      	nop
 800666c:	40005c50 	.word	0x40005c50
 8006670:	20003002 	.word	0x20003002

08006674 <GetEPDblBuf1Count>:
 8006674:	4b06      	ldr	r3, [pc, #24]	; (8006690 <GetEPDblBuf1Count+0x1c>)
 8006676:	4907      	ldr	r1, [pc, #28]	; (8006694 <GetEPDblBuf1Count+0x20>)
 8006678:	681a      	ldr	r2, [r3, #0]
 800667a:	b293      	uxth	r3, r2
 800667c:	eb03 0cc0 	add.w	ip, r3, r0, lsl #3
 8006680:	eb0c 0201 	add.w	r2, ip, r1
 8006684:	0053      	lsls	r3, r2, #1
 8006686:	6819      	ldr	r1, [r3, #0]
 8006688:	0588      	lsls	r0, r1, #22
 800668a:	0d80      	lsrs	r0, r0, #22
 800668c:	4770      	bx	lr
 800668e:	bf00      	nop
 8006690:	40005c50 	.word	0x40005c50
 8006694:	20003006 	.word	0x20003006

08006698 <GetEPDblBufDir>:
 8006698:	4b0e      	ldr	r3, [pc, #56]	; (80066d4 <GetEPDblBufDir+0x3c>)
 800669a:	00c0      	lsls	r0, r0, #3
 800669c:	6819      	ldr	r1, [r3, #0]
 800669e:	4a0e      	ldr	r2, [pc, #56]	; (80066d8 <GetEPDblBufDir+0x40>)
 80066a0:	b289      	uxth	r1, r1
 80066a2:	eb00 0c01 	add.w	ip, r0, r1
 80066a6:	4462      	add	r2, ip
 80066a8:	0051      	lsls	r1, r2, #1
 80066aa:	880a      	ldrh	r2, [r1, #0]
 80066ac:	f412 4f7c 	tst.w	r2, #64512	; 0xfc00
 80066b0:	d10e      	bne.n	80066d0 <GetEPDblBufDir+0x38>
 80066b2:	6819      	ldr	r1, [r3, #0]
 80066b4:	4b09      	ldr	r3, [pc, #36]	; (80066dc <GetEPDblBufDir+0x44>)
 80066b6:	b28a      	uxth	r2, r1
 80066b8:	eb00 0c02 	add.w	ip, r0, r2
 80066bc:	eb0c 0103 	add.w	r1, ip, r3
 80066c0:	004a      	lsls	r2, r1, #1
 80066c2:	8813      	ldrh	r3, [r2, #0]
 80066c4:	0598      	lsls	r0, r3, #22
 80066c6:	0d80      	lsrs	r0, r0, #22
 80066c8:	2800      	cmp	r0, #0
 80066ca:	bf18      	it	ne
 80066cc:	2002      	movne	r0, #2
 80066ce:	4770      	bx	lr
 80066d0:	2001      	movs	r0, #1
 80066d2:	4770      	bx	lr
 80066d4:	40005c50 	.word	0x40005c50
 80066d8:	20003006 	.word	0x20003006
 80066dc:	20003002 	.word	0x20003002

080066e0 <FreeUserBuffer>:
 80066e0:	2901      	cmp	r1, #1
 80066e2:	d014      	beq.n	800670e <FreeUserBuffer+0x2e>
 80066e4:	2902      	cmp	r1, #2
 80066e6:	d000      	beq.n	80066ea <FreeUserBuffer+0xa>
 80066e8:	4770      	bx	lr
 80066ea:	0083      	lsls	r3, r0, #2
 80066ec:	f103 4c80 	add.w	ip, r3, #1073741824	; 0x40000000
 80066f0:	f50c 4cb8 	add.w	ip, ip, #23552	; 0x5c00
 80066f4:	f8dc 2000 	ldr.w	r2, [ip]
 80066f8:	f640 710f 	movw	r1, #3855	; 0xf0f
 80066fc:	ea02 0001 	and.w	r0, r2, r1
 8006700:	f440 4340 	orr.w	r3, r0, #49152	; 0xc000
 8006704:	f043 0280 	orr.w	r2, r3, #128	; 0x80
 8006708:	f8cc 2000 	str.w	r2, [ip]
 800670c:	4770      	bx	lr
 800670e:	0081      	lsls	r1, r0, #2
 8006710:	f101 4080 	add.w	r0, r1, #1073741824	; 0x40000000
 8006714:	f500 40b8 	add.w	r0, r0, #23552	; 0x5c00
 8006718:	6803      	ldr	r3, [r0, #0]
 800671a:	f640 7c0f 	movw	ip, #3855	; 0xf0f
 800671e:	ea03 020c 	and.w	r2, r3, ip
 8006722:	f442 4100 	orr.w	r1, r2, #32768	; 0x8000
 8006726:	f041 03c0 	orr.w	r3, r1, #192	; 0xc0
 800672a:	6003      	str	r3, [r0, #0]
 800672c:	4770      	bx	lr
 800672e:	bf00      	nop

08006730 <ToWord>:
 8006730:	ea41 2000 	orr.w	r0, r1, r0, lsl #8
 8006734:	4770      	bx	lr
 8006736:	bf00      	nop

08006738 <ByteSwap>:
 8006738:	b2c3      	uxtb	r3, r0
 800673a:	0a00      	lsrs	r0, r0, #8
 800673c:	ea40 2003 	orr.w	r0, r0, r3, lsl #8
 8006740:	4770      	bx	lr
 8006742:	bf00      	nop

08006744 <USB_SIL_Init>:
 8006744:	4a04      	ldr	r2, [pc, #16]	; (8006758 <USB_SIL_Init+0x14>)
 8006746:	2000      	movs	r0, #0
 8006748:	6010      	str	r0, [r2, #0]
 800674a:	4904      	ldr	r1, [pc, #16]	; (800675c <USB_SIL_Init+0x18>)
 800674c:	4a04      	ldr	r2, [pc, #16]	; (8006760 <USB_SIL_Init+0x1c>)
 800674e:	f44f 433f 	mov.w	r3, #48896	; 0xbf00
 8006752:	800b      	strh	r3, [r1, #0]
 8006754:	6013      	str	r3, [r2, #0]
 8006756:	4770      	bx	lr
 8006758:	40005c44 	.word	0x40005c44
 800675c:	20000510 	.word	0x20000510
 8006760:	40005c40 	.word	0x40005c40

08006764 <USB_SIL_Write>:
 8006764:	b570      	push	{r4, r5, r6, lr}
 8006766:	f000 057f 	and.w	r5, r0, #127	; 0x7f
 800676a:	4628      	mov	r0, r5
 800676c:	460e      	mov	r6, r1
 800676e:	b294      	uxth	r4, r2
 8006770:	f7ff fdb8 	bl	80062e4 <GetEPTxAddr>
 8006774:	4622      	mov	r2, r4
 8006776:	4601      	mov	r1, r0
 8006778:	4630      	mov	r0, r6
 800677a:	f7ff fb47 	bl	8005e0c <UserToPMABufferCopy>
 800677e:	4628      	mov	r0, r5
 8006780:	4621      	mov	r1, r4
 8006782:	f7ff fdcf 	bl	8006324 <SetEPTxCount>
 8006786:	2000      	movs	r0, #0
 8006788:	bd70      	pop	{r4, r5, r6, pc}
 800678a:	bf00      	nop

0800678c <USB_SIL_Read>:
 800678c:	b570      	push	{r4, r5, r6, lr}
 800678e:	f000 057f 	and.w	r5, r0, #127	; 0x7f
 8006792:	4628      	mov	r0, r5
 8006794:	460e      	mov	r6, r1
 8006796:	f7ff fe21 	bl	80063dc <GetEPRxCount>
 800679a:	4604      	mov	r4, r0
 800679c:	4628      	mov	r0, r5
 800679e:	f7ff fdb1 	bl	8006304 <GetEPRxAddr>
 80067a2:	4622      	mov	r2, r4
 80067a4:	4601      	mov	r1, r0
 80067a6:	4630      	mov	r0, r6
 80067a8:	f7ff fb64 	bl	8005e74 <PMAToUserBufferCopy>
 80067ac:	4620      	mov	r0, r4
 80067ae:	bd70      	pop	{r4, r5, r6, pc}

080067b0 <__aeabi_drsub>:
 80067b0:	f081 4100 	eor.w	r1, r1, #2147483648	; 0x80000000
 80067b4:	e002      	b.n	80067bc <__adddf3>
 80067b6:	bf00      	nop

080067b8 <__aeabi_dsub>:
 80067b8:	f083 4300 	eor.w	r3, r3, #2147483648	; 0x80000000

080067bc <__adddf3>:
 80067bc:	b530      	push	{r4, r5, lr}
 80067be:	ea4f 0441 	mov.w	r4, r1, lsl #1
 80067c2:	ea4f 0543 	mov.w	r5, r3, lsl #1
 80067c6:	ea94 0f05 	teq	r4, r5
 80067ca:	bf08      	it	eq
 80067cc:	ea90 0f02 	teqeq	r0, r2
 80067d0:	bf1f      	itttt	ne
 80067d2:	ea54 0c00 	orrsne.w	ip, r4, r0
 80067d6:	ea55 0c02 	orrsne.w	ip, r5, r2
 80067da:	ea7f 5c64 	mvnsne.w	ip, r4, asr #21
 80067de:	ea7f 5c65 	mvnsne.w	ip, r5, asr #21
 80067e2:	f000 80e2 	beq.w	80069aa <__adddf3+0x1ee>
 80067e6:	ea4f 5454 	mov.w	r4, r4, lsr #21
 80067ea:	ebd4 5555 	rsbs	r5, r4, r5, lsr #21
 80067ee:	bfb8      	it	lt
 80067f0:	426d      	neglt	r5, r5
 80067f2:	dd0c      	ble.n	800680e <__adddf3+0x52>
 80067f4:	442c      	add	r4, r5
 80067f6:	ea80 0202 	eor.w	r2, r0, r2
 80067fa:	ea81 0303 	eor.w	r3, r1, r3
 80067fe:	ea82 0000 	eor.w	r0, r2, r0
 8006802:	ea83 0101 	eor.w	r1, r3, r1
 8006806:	ea80 0202 	eor.w	r2, r0, r2
 800680a:	ea81 0303 	eor.w	r3, r1, r3
 800680e:	2d36      	cmp	r5, #54	; 0x36
 8006810:	bf88      	it	hi
 8006812:	bd30      	pophi	{r4, r5, pc}
 8006814:	f011 4f00 	tst.w	r1, #2147483648	; 0x80000000
 8006818:	ea4f 3101 	mov.w	r1, r1, lsl #12
 800681c:	f44f 1c80 	mov.w	ip, #1048576	; 0x100000
 8006820:	ea4c 3111 	orr.w	r1, ip, r1, lsr #12
 8006824:	d002      	beq.n	800682c <__adddf3+0x70>
 8006826:	4240      	negs	r0, r0
 8006828:	eb61 0141 	sbc.w	r1, r1, r1, lsl #1
 800682c:	f013 4f00 	tst.w	r3, #2147483648	; 0x80000000
 8006830:	ea4f 3303 	mov.w	r3, r3, lsl #12
 8006834:	ea4c 3313 	orr.w	r3, ip, r3, lsr #12
 8006838:	d002      	beq.n	8006840 <__adddf3+0x84>
 800683a:	4252      	negs	r2, r2
 800683c:	eb63 0343 	sbc.w	r3, r3, r3, lsl #1
 8006840:	ea94 0f05 	teq	r4, r5
 8006844:	f000 80a7 	beq.w	8006996 <__adddf3+0x1da>
 8006848:	f1a4 0401 	sub.w	r4, r4, #1
 800684c:	f1d5 0e20 	rsbs	lr, r5, #32
 8006850:	db0d      	blt.n	800686e <__adddf3+0xb2>
 8006852:	fa02 fc0e 	lsl.w	ip, r2, lr
 8006856:	fa22 f205 	lsr.w	r2, r2, r5
 800685a:	1880      	adds	r0, r0, r2
 800685c:	f141 0100 	adc.w	r1, r1, #0
 8006860:	fa03 f20e 	lsl.w	r2, r3, lr
 8006864:	1880      	adds	r0, r0, r2
 8006866:	fa43 f305 	asr.w	r3, r3, r5
 800686a:	4159      	adcs	r1, r3
 800686c:	e00e      	b.n	800688c <__adddf3+0xd0>
 800686e:	f1a5 0520 	sub.w	r5, r5, #32
 8006872:	f10e 0e20 	add.w	lr, lr, #32
 8006876:	2a01      	cmp	r2, #1
 8006878:	fa03 fc0e 	lsl.w	ip, r3, lr
 800687c:	bf28      	it	cs
 800687e:	f04c 0c02 	orrcs.w	ip, ip, #2
 8006882:	fa43 f305 	asr.w	r3, r3, r5
 8006886:	18c0      	adds	r0, r0, r3
 8006888:	eb51 71e3 	adcs.w	r1, r1, r3, asr #31
 800688c:	f001 4500 	and.w	r5, r1, #2147483648	; 0x80000000
 8006890:	d507      	bpl.n	80068a2 <__adddf3+0xe6>
 8006892:	f04f 0e00 	mov.w	lr, #0
 8006896:	f1dc 0c00 	rsbs	ip, ip, #0
 800689a:	eb7e 0000 	sbcs.w	r0, lr, r0
 800689e:	eb6e 0101 	sbc.w	r1, lr, r1
 80068a2:	f5b1 1f80 	cmp.w	r1, #1048576	; 0x100000
 80068a6:	d31b      	bcc.n	80068e0 <__adddf3+0x124>
 80068a8:	f5b1 1f00 	cmp.w	r1, #2097152	; 0x200000
 80068ac:	d30c      	bcc.n	80068c8 <__adddf3+0x10c>
 80068ae:	0849      	lsrs	r1, r1, #1
 80068b0:	ea5f 0030 	movs.w	r0, r0, rrx
 80068b4:	ea4f 0c3c 	mov.w	ip, ip, rrx
 80068b8:	f104 0401 	add.w	r4, r4, #1
 80068bc:	ea4f 5244 	mov.w	r2, r4, lsl #21
 80068c0:	f512 0f80 	cmn.w	r2, #4194304	; 0x400000
 80068c4:	f080 809a 	bcs.w	80069fc <__adddf3+0x240>
 80068c8:	f1bc 4f00 	cmp.w	ip, #2147483648	; 0x80000000
 80068cc:	bf08      	it	eq
 80068ce:	ea5f 0c50 	movseq.w	ip, r0, lsr #1
 80068d2:	f150 0000 	adcs.w	r0, r0, #0
 80068d6:	eb41 5104 	adc.w	r1, r1, r4, lsl #20
 80068da:	ea41 0105 	orr.w	r1, r1, r5
 80068de:	bd30      	pop	{r4, r5, pc}
 80068e0:	ea5f 0c4c 	movs.w	ip, ip, lsl #1
 80068e4:	4140      	adcs	r0, r0
 80068e6:	eb41 0101 	adc.w	r1, r1, r1
 80068ea:	f411 1f80 	tst.w	r1, #1048576	; 0x100000
 80068ee:	f1a4 0401 	sub.w	r4, r4, #1
 80068f2:	d1e9      	bne.n	80068c8 <__adddf3+0x10c>
 80068f4:	f091 0f00 	teq	r1, #0
 80068f8:	bf04      	itt	eq
 80068fa:	4601      	moveq	r1, r0
 80068fc:	2000      	moveq	r0, #0
 80068fe:	fab1 f381 	clz	r3, r1
 8006902:	bf08      	it	eq
 8006904:	3320      	addeq	r3, #32
 8006906:	f1a3 030b 	sub.w	r3, r3, #11
 800690a:	f1b3 0220 	subs.w	r2, r3, #32
 800690e:	da0c      	bge.n	800692a <__adddf3+0x16e>
 8006910:	320c      	adds	r2, #12
 8006912:	dd08      	ble.n	8006926 <__adddf3+0x16a>
 8006914:	f102 0c14 	add.w	ip, r2, #20
 8006918:	f1c2 020c 	rsb	r2, r2, #12
 800691c:	fa01 f00c 	lsl.w	r0, r1, ip
 8006920:	fa21 f102 	lsr.w	r1, r1, r2
 8006924:	e00c      	b.n	8006940 <__adddf3+0x184>
 8006926:	f102 0214 	add.w	r2, r2, #20
 800692a:	bfd8      	it	le
 800692c:	f1c2 0c20 	rsble	ip, r2, #32
 8006930:	fa01 f102 	lsl.w	r1, r1, r2
 8006934:	fa20 fc0c 	lsr.w	ip, r0, ip
 8006938:	bfdc      	itt	le
 800693a:	ea41 010c 	orrle.w	r1, r1, ip
 800693e:	4090      	lslle	r0, r2
 8006940:	1ae4      	subs	r4, r4, r3
 8006942:	bfa2      	ittt	ge
 8006944:	eb01 5104 	addge.w	r1, r1, r4, lsl #20
 8006948:	4329      	orrge	r1, r5
 800694a:	bd30      	popge	{r4, r5, pc}
 800694c:	ea6f 0404 	mvn.w	r4, r4
 8006950:	3c1f      	subs	r4, #31
 8006952:	da1c      	bge.n	800698e <__adddf3+0x1d2>
 8006954:	340c      	adds	r4, #12
 8006956:	dc0e      	bgt.n	8006976 <__adddf3+0x1ba>
 8006958:	f104 0414 	add.w	r4, r4, #20
 800695c:	f1c4 0220 	rsb	r2, r4, #32
 8006960:	fa20 f004 	lsr.w	r0, r0, r4
 8006964:	fa01 f302 	lsl.w	r3, r1, r2
 8006968:	ea40 0003 	orr.w	r0, r0, r3
 800696c:	fa21 f304 	lsr.w	r3, r1, r4
 8006970:	ea45 0103 	orr.w	r1, r5, r3
 8006974:	bd30      	pop	{r4, r5, pc}
 8006976:	f1c4 040c 	rsb	r4, r4, #12
 800697a:	f1c4 0220 	rsb	r2, r4, #32
 800697e:	fa20 f002 	lsr.w	r0, r0, r2
 8006982:	fa01 f304 	lsl.w	r3, r1, r4
 8006986:	ea40 0003 	orr.w	r0, r0, r3
 800698a:	4629      	mov	r1, r5
 800698c:	bd30      	pop	{r4, r5, pc}
 800698e:	fa21 f004 	lsr.w	r0, r1, r4
 8006992:	4629      	mov	r1, r5
 8006994:	bd30      	pop	{r4, r5, pc}
 8006996:	f094 0f00 	teq	r4, #0
 800699a:	f483 1380 	eor.w	r3, r3, #1048576	; 0x100000
 800699e:	bf06      	itte	eq
 80069a0:	f481 1180 	eoreq.w	r1, r1, #1048576	; 0x100000
 80069a4:	3401      	addeq	r4, #1
 80069a6:	3d01      	subne	r5, #1
 80069a8:	e74e      	b.n	8006848 <__adddf3+0x8c>
 80069aa:	ea7f 5c64 	mvns.w	ip, r4, asr #21
 80069ae:	bf18      	it	ne
 80069b0:	ea7f 5c65 	mvnsne.w	ip, r5, asr #21
 80069b4:	d029      	beq.n	8006a0a <__adddf3+0x24e>
 80069b6:	ea94 0f05 	teq	r4, r5
 80069ba:	bf08      	it	eq
 80069bc:	ea90 0f02 	teqeq	r0, r2
 80069c0:	d005      	beq.n	80069ce <__adddf3+0x212>
 80069c2:	ea54 0c00 	orrs.w	ip, r4, r0
 80069c6:	bf04      	itt	eq
 80069c8:	4619      	moveq	r1, r3
 80069ca:	4610      	moveq	r0, r2
 80069cc:	bd30      	pop	{r4, r5, pc}
 80069ce:	ea91 0f03 	teq	r1, r3
 80069d2:	bf1e      	ittt	ne
 80069d4:	2100      	movne	r1, #0
 80069d6:	2000      	movne	r0, #0
 80069d8:	bd30      	popne	{r4, r5, pc}
 80069da:	ea5f 5c54 	movs.w	ip, r4, lsr #21
 80069de:	d105      	bne.n	80069ec <__adddf3+0x230>
 80069e0:	0040      	lsls	r0, r0, #1
 80069e2:	4149      	adcs	r1, r1
 80069e4:	bf28      	it	cs
 80069e6:	f041 4100 	orrcs.w	r1, r1, #2147483648	; 0x80000000
 80069ea:	bd30      	pop	{r4, r5, pc}
 80069ec:	f514 0480 	adds.w	r4, r4, #4194304	; 0x400000
 80069f0:	bf3c      	itt	cc
 80069f2:	f501 1180 	addcc.w	r1, r1, #1048576	; 0x100000
 80069f6:	bd30      	popcc	{r4, r5, pc}
 80069f8:	f001 4500 	and.w	r5, r1, #2147483648	; 0x80000000
 80069fc:	f045 41fe 	orr.w	r1, r5, #2130706432	; 0x7f000000
 8006a00:	f441 0170 	orr.w	r1, r1, #15728640	; 0xf00000
 8006a04:	f04f 0000 	mov.w	r0, #0
 8006a08:	bd30      	pop	{r4, r5, pc}
 8006a0a:	ea7f 5c64 	mvns.w	ip, r4, asr #21
 8006a0e:	bf1a      	itte	ne
 8006a10:	4619      	movne	r1, r3
 8006a12:	4610      	movne	r0, r2
 8006a14:	ea7f 5c65 	mvnseq.w	ip, r5, asr #21
 8006a18:	bf1c      	itt	ne
 8006a1a:	460b      	movne	r3, r1
 8006a1c:	4602      	movne	r2, r0
 8006a1e:	ea50 3401 	orrs.w	r4, r0, r1, lsl #12
 8006a22:	bf06      	itte	eq
 8006a24:	ea52 3503 	orrseq.w	r5, r2, r3, lsl #12
 8006a28:	ea91 0f03 	teqeq	r1, r3
 8006a2c:	f441 2100 	orrne.w	r1, r1, #524288	; 0x80000
 8006a30:	bd30      	pop	{r4, r5, pc}
 8006a32:	bf00      	nop

08006a34 <__aeabi_ui2d>:
 8006a34:	f090 0f00 	teq	r0, #0
 8006a38:	bf04      	itt	eq
 8006a3a:	2100      	moveq	r1, #0
 8006a3c:	4770      	bxeq	lr
 8006a3e:	b530      	push	{r4, r5, lr}
 8006a40:	f44f 6480 	mov.w	r4, #1024	; 0x400
 8006a44:	f104 0432 	add.w	r4, r4, #50	; 0x32
 8006a48:	f04f 0500 	mov.w	r5, #0
 8006a4c:	f04f 0100 	mov.w	r1, #0
 8006a50:	e750      	b.n	80068f4 <__adddf3+0x138>
 8006a52:	bf00      	nop

08006a54 <__aeabi_i2d>:
 8006a54:	f090 0f00 	teq	r0, #0
 8006a58:	bf04      	itt	eq
 8006a5a:	2100      	moveq	r1, #0
 8006a5c:	4770      	bxeq	lr
 8006a5e:	b530      	push	{r4, r5, lr}
 8006a60:	f44f 6480 	mov.w	r4, #1024	; 0x400
 8006a64:	f104 0432 	add.w	r4, r4, #50	; 0x32
 8006a68:	f010 4500 	ands.w	r5, r0, #2147483648	; 0x80000000
 8006a6c:	bf48      	it	mi
 8006a6e:	4240      	negmi	r0, r0
 8006a70:	f04f 0100 	mov.w	r1, #0
 8006a74:	e73e      	b.n	80068f4 <__adddf3+0x138>
 8006a76:	bf00      	nop

08006a78 <__aeabi_f2d>:
 8006a78:	0042      	lsls	r2, r0, #1
 8006a7a:	ea4f 01e2 	mov.w	r1, r2, asr #3
 8006a7e:	ea4f 0131 	mov.w	r1, r1, rrx
 8006a82:	ea4f 7002 	mov.w	r0, r2, lsl #28
 8006a86:	bf1f      	itttt	ne
 8006a88:	f012 437f 	andsne.w	r3, r2, #4278190080	; 0xff000000
 8006a8c:	f093 4f7f 	teqne	r3, #4278190080	; 0xff000000
 8006a90:	f081 5160 	eorne.w	r1, r1, #939524096	; 0x38000000
 8006a94:	4770      	bxne	lr
 8006a96:	f092 0f00 	teq	r2, #0
 8006a9a:	bf14      	ite	ne
 8006a9c:	f093 4f7f 	teqne	r3, #4278190080	; 0xff000000
 8006aa0:	4770      	bxeq	lr
 8006aa2:	b530      	push	{r4, r5, lr}
 8006aa4:	f44f 7460 	mov.w	r4, #896	; 0x380
 8006aa8:	f001 4500 	and.w	r5, r1, #2147483648	; 0x80000000
 8006aac:	f021 4100 	bic.w	r1, r1, #2147483648	; 0x80000000
 8006ab0:	e720      	b.n	80068f4 <__adddf3+0x138>
 8006ab2:	bf00      	nop

08006ab4 <__aeabi_ul2d>:
 8006ab4:	ea50 0201 	orrs.w	r2, r0, r1
 8006ab8:	bf08      	it	eq
 8006aba:	4770      	bxeq	lr
 8006abc:	b530      	push	{r4, r5, lr}
 8006abe:	f04f 0500 	mov.w	r5, #0
 8006ac2:	e00a      	b.n	8006ada <__aeabi_l2d+0x16>

08006ac4 <__aeabi_l2d>:
 8006ac4:	ea50 0201 	orrs.w	r2, r0, r1
 8006ac8:	bf08      	it	eq
 8006aca:	4770      	bxeq	lr
 8006acc:	b530      	push	{r4, r5, lr}
 8006ace:	f011 4500 	ands.w	r5, r1, #2147483648	; 0x80000000
 8006ad2:	d502      	bpl.n	8006ada <__aeabi_l2d+0x16>
 8006ad4:	4240      	negs	r0, r0
 8006ad6:	eb61 0141 	sbc.w	r1, r1, r1, lsl #1
 8006ada:	f44f 6480 	mov.w	r4, #1024	; 0x400
 8006ade:	f104 0432 	add.w	r4, r4, #50	; 0x32
 8006ae2:	ea5f 5c91 	movs.w	ip, r1, lsr #22
 8006ae6:	f43f aedc 	beq.w	80068a2 <__adddf3+0xe6>
 8006aea:	f04f 0203 	mov.w	r2, #3
 8006aee:	ea5f 0cdc 	movs.w	ip, ip, lsr #3
 8006af2:	bf18      	it	ne
 8006af4:	3203      	addne	r2, #3
 8006af6:	ea5f 0cdc 	movs.w	ip, ip, lsr #3
 8006afa:	bf18      	it	ne
 8006afc:	3203      	addne	r2, #3
 8006afe:	eb02 02dc 	add.w	r2, r2, ip, lsr #3
 8006b02:	f1c2 0320 	rsb	r3, r2, #32
 8006b06:	fa00 fc03 	lsl.w	ip, r0, r3
 8006b0a:	fa20 f002 	lsr.w	r0, r0, r2
 8006b0e:	fa01 fe03 	lsl.w	lr, r1, r3
 8006b12:	ea40 000e 	orr.w	r0, r0, lr
 8006b16:	fa21 f102 	lsr.w	r1, r1, r2
 8006b1a:	4414      	add	r4, r2
 8006b1c:	e6c1      	b.n	80068a2 <__adddf3+0xe6>
 8006b1e:	bf00      	nop

08006b20 <__aeabi_dmul>:
 8006b20:	b570      	push	{r4, r5, r6, lr}
 8006b22:	f04f 0cff 	mov.w	ip, #255	; 0xff
 8006b26:	f44c 6ce0 	orr.w	ip, ip, #1792	; 0x700
 8006b2a:	ea1c 5411 	ands.w	r4, ip, r1, lsr #20
 8006b2e:	bf1d      	ittte	ne
 8006b30:	ea1c 5513 	andsne.w	r5, ip, r3, lsr #20
 8006b34:	ea94 0f0c 	teqne	r4, ip
 8006b38:	ea95 0f0c 	teqne	r5, ip
 8006b3c:	f000 f8de 	bleq	8006cfc <__aeabi_dmul+0x1dc>
 8006b40:	442c      	add	r4, r5
 8006b42:	ea81 0603 	eor.w	r6, r1, r3
 8006b46:	ea21 514c 	bic.w	r1, r1, ip, lsl #21
 8006b4a:	ea23 534c 	bic.w	r3, r3, ip, lsl #21
 8006b4e:	ea50 3501 	orrs.w	r5, r0, r1, lsl #12
 8006b52:	bf18      	it	ne
 8006b54:	ea52 3503 	orrsne.w	r5, r2, r3, lsl #12
 8006b58:	f441 1180 	orr.w	r1, r1, #1048576	; 0x100000
 8006b5c:	f443 1380 	orr.w	r3, r3, #1048576	; 0x100000
 8006b60:	d038      	beq.n	8006bd4 <__aeabi_dmul+0xb4>
 8006b62:	fba0 ce02 	umull	ip, lr, r0, r2
 8006b66:	f04f 0500 	mov.w	r5, #0
 8006b6a:	fbe1 e502 	umlal	lr, r5, r1, r2
 8006b6e:	f006 4200 	and.w	r2, r6, #2147483648	; 0x80000000
 8006b72:	fbe0 e503 	umlal	lr, r5, r0, r3
 8006b76:	f04f 0600 	mov.w	r6, #0
 8006b7a:	fbe1 5603 	umlal	r5, r6, r1, r3
 8006b7e:	f09c 0f00 	teq	ip, #0
 8006b82:	bf18      	it	ne
 8006b84:	f04e 0e01 	orrne.w	lr, lr, #1
 8006b88:	f1a4 04ff 	sub.w	r4, r4, #255	; 0xff
 8006b8c:	f5b6 7f00 	cmp.w	r6, #512	; 0x200
 8006b90:	f564 7440 	sbc.w	r4, r4, #768	; 0x300
 8006b94:	d204      	bcs.n	8006ba0 <__aeabi_dmul+0x80>
 8006b96:	ea5f 0e4e 	movs.w	lr, lr, lsl #1
 8006b9a:	416d      	adcs	r5, r5
 8006b9c:	eb46 0606 	adc.w	r6, r6, r6
 8006ba0:	ea42 21c6 	orr.w	r1, r2, r6, lsl #11
 8006ba4:	ea41 5155 	orr.w	r1, r1, r5, lsr #21
 8006ba8:	ea4f 20c5 	mov.w	r0, r5, lsl #11
 8006bac:	ea40 505e 	orr.w	r0, r0, lr, lsr #21
 8006bb0:	ea4f 2ece 	mov.w	lr, lr, lsl #11
 8006bb4:	f1b4 0cfd 	subs.w	ip, r4, #253	; 0xfd
 8006bb8:	bf88      	it	hi
 8006bba:	f5bc 6fe0 	cmphi.w	ip, #1792	; 0x700
 8006bbe:	d81e      	bhi.n	8006bfe <__aeabi_dmul+0xde>
 8006bc0:	f1be 4f00 	cmp.w	lr, #2147483648	; 0x80000000
 8006bc4:	bf08      	it	eq
 8006bc6:	ea5f 0e50 	movseq.w	lr, r0, lsr #1
 8006bca:	f150 0000 	adcs.w	r0, r0, #0
 8006bce:	eb41 5104 	adc.w	r1, r1, r4, lsl #20
 8006bd2:	bd70      	pop	{r4, r5, r6, pc}
 8006bd4:	f006 4600 	and.w	r6, r6, #2147483648	; 0x80000000
 8006bd8:	ea46 0101 	orr.w	r1, r6, r1
 8006bdc:	ea40 0002 	orr.w	r0, r0, r2
 8006be0:	ea81 0103 	eor.w	r1, r1, r3
 8006be4:	ebb4 045c 	subs.w	r4, r4, ip, lsr #1
 8006be8:	bfc2      	ittt	gt
 8006bea:	ebd4 050c 	rsbsgt	r5, r4, ip
 8006bee:	ea41 5104 	orrgt.w	r1, r1, r4, lsl #20
 8006bf2:	bd70      	popgt	{r4, r5, r6, pc}
 8006bf4:	f441 1180 	orr.w	r1, r1, #1048576	; 0x100000
 8006bf8:	f04f 0e00 	mov.w	lr, #0
 8006bfc:	3c01      	subs	r4, #1
 8006bfe:	f300 80ab 	bgt.w	8006d58 <__aeabi_dmul+0x238>
 8006c02:	f114 0f36 	cmn.w	r4, #54	; 0x36
 8006c06:	bfde      	ittt	le
 8006c08:	2000      	movle	r0, #0
 8006c0a:	f001 4100 	andle.w	r1, r1, #2147483648	; 0x80000000
 8006c0e:	bd70      	pople	{r4, r5, r6, pc}
 8006c10:	f1c4 0400 	rsb	r4, r4, #0
 8006c14:	3c20      	subs	r4, #32
 8006c16:	da35      	bge.n	8006c84 <__aeabi_dmul+0x164>
 8006c18:	340c      	adds	r4, #12
 8006c1a:	dc1b      	bgt.n	8006c54 <__aeabi_dmul+0x134>
 8006c1c:	f104 0414 	add.w	r4, r4, #20
 8006c20:	f1c4 0520 	rsb	r5, r4, #32
 8006c24:	fa00 f305 	lsl.w	r3, r0, r5
 8006c28:	fa20 f004 	lsr.w	r0, r0, r4
 8006c2c:	fa01 f205 	lsl.w	r2, r1, r5
 8006c30:	ea40 0002 	orr.w	r0, r0, r2
 8006c34:	f001 4200 	and.w	r2, r1, #2147483648	; 0x80000000
 8006c38:	f021 4100 	bic.w	r1, r1, #2147483648	; 0x80000000
 8006c3c:	eb10 70d3 	adds.w	r0, r0, r3, lsr #31
 8006c40:	fa21 f604 	lsr.w	r6, r1, r4
 8006c44:	eb42 0106 	adc.w	r1, r2, r6
 8006c48:	ea5e 0e43 	orrs.w	lr, lr, r3, lsl #1
 8006c4c:	bf08      	it	eq
 8006c4e:	ea20 70d3 	biceq.w	r0, r0, r3, lsr #31
 8006c52:	bd70      	pop	{r4, r5, r6, pc}
 8006c54:	f1c4 040c 	rsb	r4, r4, #12
 8006c58:	f1c4 0520 	rsb	r5, r4, #32
 8006c5c:	fa00 f304 	lsl.w	r3, r0, r4
 8006c60:	fa20 f005 	lsr.w	r0, r0, r5
 8006c64:	fa01 f204 	lsl.w	r2, r1, r4
 8006c68:	ea40 0002 	orr.w	r0, r0, r2
 8006c6c:	f001 4100 	and.w	r1, r1, #2147483648	; 0x80000000
 8006c70:	eb10 70d3 	adds.w	r0, r0, r3, lsr #31
 8006c74:	f141 0100 	adc.w	r1, r1, #0
 8006c78:	ea5e 0e43 	orrs.w	lr, lr, r3, lsl #1
 8006c7c:	bf08      	it	eq
 8006c7e:	ea20 70d3 	biceq.w	r0, r0, r3, lsr #31
 8006c82:	bd70      	pop	{r4, r5, r6, pc}
 8006c84:	f1c4 0520 	rsb	r5, r4, #32
 8006c88:	fa00 f205 	lsl.w	r2, r0, r5
 8006c8c:	ea4e 0e02 	orr.w	lr, lr, r2
 8006c90:	fa20 f304 	lsr.w	r3, r0, r4
 8006c94:	fa01 f205 	lsl.w	r2, r1, r5
 8006c98:	ea43 0302 	orr.w	r3, r3, r2
 8006c9c:	fa21 f004 	lsr.w	r0, r1, r4
 8006ca0:	f001 4100 	and.w	r1, r1, #2147483648	; 0x80000000
 8006ca4:	fa21 f204 	lsr.w	r2, r1, r4
 8006ca8:	ea20 0002 	bic.w	r0, r0, r2
 8006cac:	eb00 70d3 	add.w	r0, r0, r3, lsr #31
 8006cb0:	ea5e 0e43 	orrs.w	lr, lr, r3, lsl #1
 8006cb4:	bf08      	it	eq
 8006cb6:	ea20 70d3 	biceq.w	r0, r0, r3, lsr #31
 8006cba:	bd70      	pop	{r4, r5, r6, pc}
 8006cbc:	f094 0f00 	teq	r4, #0
 8006cc0:	d10f      	bne.n	8006ce2 <__aeabi_dmul+0x1c2>
 8006cc2:	f001 4600 	and.w	r6, r1, #2147483648	; 0x80000000
 8006cc6:	0040      	lsls	r0, r0, #1
 8006cc8:	eb41 0101 	adc.w	r1, r1, r1
 8006ccc:	f411 1f80 	tst.w	r1, #1048576	; 0x100000
 8006cd0:	bf08      	it	eq
 8006cd2:	3c01      	subeq	r4, #1
 8006cd4:	d0f7      	beq.n	8006cc6 <__aeabi_dmul+0x1a6>
 8006cd6:	ea41 0106 	orr.w	r1, r1, r6
 8006cda:	f095 0f00 	teq	r5, #0
 8006cde:	bf18      	it	ne
 8006ce0:	4770      	bxne	lr
 8006ce2:	f003 4600 	and.w	r6, r3, #2147483648	; 0x80000000
 8006ce6:	0052      	lsls	r2, r2, #1
 8006ce8:	eb43 0303 	adc.w	r3, r3, r3
 8006cec:	f413 1f80 	tst.w	r3, #1048576	; 0x100000
 8006cf0:	bf08      	it	eq
 8006cf2:	3d01      	subeq	r5, #1
 8006cf4:	d0f7      	beq.n	8006ce6 <__aeabi_dmul+0x1c6>
 8006cf6:	ea43 0306 	orr.w	r3, r3, r6
 8006cfa:	4770      	bx	lr
 8006cfc:	ea94 0f0c 	teq	r4, ip
 8006d00:	ea0c 5513 	and.w	r5, ip, r3, lsr #20
 8006d04:	bf18      	it	ne
 8006d06:	ea95 0f0c 	teqne	r5, ip
 8006d0a:	d00c      	beq.n	8006d26 <__aeabi_dmul+0x206>
 8006d0c:	ea50 0641 	orrs.w	r6, r0, r1, lsl #1
 8006d10:	bf18      	it	ne
 8006d12:	ea52 0643 	orrsne.w	r6, r2, r3, lsl #1
 8006d16:	d1d1      	bne.n	8006cbc <__aeabi_dmul+0x19c>
 8006d18:	ea81 0103 	eor.w	r1, r1, r3
 8006d1c:	f001 4100 	and.w	r1, r1, #2147483648	; 0x80000000
 8006d20:	f04f 0000 	mov.w	r0, #0
 8006d24:	bd70      	pop	{r4, r5, r6, pc}
 8006d26:	ea50 0641 	orrs.w	r6, r0, r1, lsl #1
 8006d2a:	bf06      	itte	eq
 8006d2c:	4610      	moveq	r0, r2
 8006d2e:	4619      	moveq	r1, r3
 8006d30:	ea52 0643 	orrsne.w	r6, r2, r3, lsl #1
 8006d34:	d019      	beq.n	8006d6a <__aeabi_dmul+0x24a>
 8006d36:	ea94 0f0c 	teq	r4, ip
 8006d3a:	d102      	bne.n	8006d42 <__aeabi_dmul+0x222>
 8006d3c:	ea50 3601 	orrs.w	r6, r0, r1, lsl #12
 8006d40:	d113      	bne.n	8006d6a <__aeabi_dmul+0x24a>
 8006d42:	ea95 0f0c 	teq	r5, ip
 8006d46:	d105      	bne.n	8006d54 <__aeabi_dmul+0x234>
 8006d48:	ea52 3603 	orrs.w	r6, r2, r3, lsl #12
 8006d4c:	bf1c      	itt	ne
 8006d4e:	4610      	movne	r0, r2
 8006d50:	4619      	movne	r1, r3
 8006d52:	d10a      	bne.n	8006d6a <__aeabi_dmul+0x24a>
 8006d54:	ea81 0103 	eor.w	r1, r1, r3
 8006d58:	f001 4100 	and.w	r1, r1, #2147483648	; 0x80000000
 8006d5c:	f041 41fe 	orr.w	r1, r1, #2130706432	; 0x7f000000
 8006d60:	f441 0170 	orr.w	r1, r1, #15728640	; 0xf00000
 8006d64:	f04f 0000 	mov.w	r0, #0
 8006d68:	bd70      	pop	{r4, r5, r6, pc}
 8006d6a:	f041 41fe 	orr.w	r1, r1, #2130706432	; 0x7f000000
 8006d6e:	f441 0178 	orr.w	r1, r1, #16252928	; 0xf80000
 8006d72:	bd70      	pop	{r4, r5, r6, pc}

08006d74 <__aeabi_ddiv>:
 8006d74:	b570      	push	{r4, r5, r6, lr}
 8006d76:	f04f 0cff 	mov.w	ip, #255	; 0xff
 8006d7a:	f44c 6ce0 	orr.w	ip, ip, #1792	; 0x700
 8006d7e:	ea1c 5411 	ands.w	r4, ip, r1, lsr #20
 8006d82:	bf1d      	ittte	ne
 8006d84:	ea1c 5513 	andsne.w	r5, ip, r3, lsr #20
 8006d88:	ea94 0f0c 	teqne	r4, ip
 8006d8c:	ea95 0f0c 	teqne	r5, ip
 8006d90:	f000 f8a7 	bleq	8006ee2 <__aeabi_ddiv+0x16e>
 8006d94:	eba4 0405 	sub.w	r4, r4, r5
 8006d98:	ea81 0e03 	eor.w	lr, r1, r3
 8006d9c:	ea52 3503 	orrs.w	r5, r2, r3, lsl #12
 8006da0:	ea4f 3101 	mov.w	r1, r1, lsl #12
 8006da4:	f000 8088 	beq.w	8006eb8 <__aeabi_ddiv+0x144>
 8006da8:	ea4f 3303 	mov.w	r3, r3, lsl #12
 8006dac:	f04f 5580 	mov.w	r5, #268435456	; 0x10000000
 8006db0:	ea45 1313 	orr.w	r3, r5, r3, lsr #4
 8006db4:	ea43 6312 	orr.w	r3, r3, r2, lsr #24
 8006db8:	ea4f 2202 	mov.w	r2, r2, lsl #8
 8006dbc:	ea45 1511 	orr.w	r5, r5, r1, lsr #4
 8006dc0:	ea45 6510 	orr.w	r5, r5, r0, lsr #24
 8006dc4:	ea4f 2600 	mov.w	r6, r0, lsl #8
 8006dc8:	f00e 4100 	and.w	r1, lr, #2147483648	; 0x80000000
 8006dcc:	429d      	cmp	r5, r3
 8006dce:	bf08      	it	eq
 8006dd0:	4296      	cmpeq	r6, r2
 8006dd2:	f144 04fd 	adc.w	r4, r4, #253	; 0xfd
 8006dd6:	f504 7440 	add.w	r4, r4, #768	; 0x300
 8006dda:	d202      	bcs.n	8006de2 <__aeabi_ddiv+0x6e>
 8006ddc:	085b      	lsrs	r3, r3, #1
 8006dde:	ea4f 0232 	mov.w	r2, r2, rrx
 8006de2:	1ab6      	subs	r6, r6, r2
 8006de4:	eb65 0503 	sbc.w	r5, r5, r3
 8006de8:	085b      	lsrs	r3, r3, #1
 8006dea:	ea4f 0232 	mov.w	r2, r2, rrx
 8006dee:	f44f 1080 	mov.w	r0, #1048576	; 0x100000
 8006df2:	f44f 2c00 	mov.w	ip, #524288	; 0x80000
 8006df6:	ebb6 0e02 	subs.w	lr, r6, r2
 8006dfa:	eb75 0e03 	sbcs.w	lr, r5, r3
 8006dfe:	bf22      	ittt	cs
 8006e00:	1ab6      	subcs	r6, r6, r2
 8006e02:	4675      	movcs	r5, lr
 8006e04:	ea40 000c 	orrcs.w	r0, r0, ip
 8006e08:	085b      	lsrs	r3, r3, #1
 8006e0a:	ea4f 0232 	mov.w	r2, r2, rrx
 8006e0e:	ebb6 0e02 	subs.w	lr, r6, r2
 8006e12:	eb75 0e03 	sbcs.w	lr, r5, r3
 8006e16:	bf22      	ittt	cs
 8006e18:	1ab6      	subcs	r6, r6, r2
 8006e1a:	4675      	movcs	r5, lr
 8006e1c:	ea40 005c 	orrcs.w	r0, r0, ip, lsr #1
 8006e20:	085b      	lsrs	r3, r3, #1
 8006e22:	ea4f 0232 	mov.w	r2, r2, rrx
 8006e26:	ebb6 0e02 	subs.w	lr, r6, r2
 8006e2a:	eb75 0e03 	sbcs.w	lr, r5, r3
 8006e2e:	bf22      	ittt	cs
 8006e30:	1ab6      	subcs	r6, r6, r2
 8006e32:	4675      	movcs	r5, lr
 8006e34:	ea40 009c 	orrcs.w	r0, r0, ip, lsr #2
 8006e38:	085b      	lsrs	r3, r3, #1
 8006e3a:	ea4f 0232 	mov.w	r2, r2, rrx
 8006e3e:	ebb6 0e02 	subs.w	lr, r6, r2
 8006e42:	eb75 0e03 	sbcs.w	lr, r5, r3
 8006e46:	bf22      	ittt	cs
 8006e48:	1ab6      	subcs	r6, r6, r2
 8006e4a:	4675      	movcs	r5, lr
 8006e4c:	ea40 00dc 	orrcs.w	r0, r0, ip, lsr #3
 8006e50:	ea55 0e06 	orrs.w	lr, r5, r6
 8006e54:	d018      	beq.n	8006e88 <__aeabi_ddiv+0x114>
 8006e56:	ea4f 1505 	mov.w	r5, r5, lsl #4
 8006e5a:	ea45 7516 	orr.w	r5, r5, r6, lsr #28
 8006e5e:	ea4f 1606 	mov.w	r6, r6, lsl #4
 8006e62:	ea4f 03c3 	mov.w	r3, r3, lsl #3
 8006e66:	ea43 7352 	orr.w	r3, r3, r2, lsr #29
 8006e6a:	ea4f 02c2 	mov.w	r2, r2, lsl #3
 8006e6e:	ea5f 1c1c 	movs.w	ip, ip, lsr #4
 8006e72:	d1c0      	bne.n	8006df6 <__aeabi_ddiv+0x82>
 8006e74:	f411 1f80 	tst.w	r1, #1048576	; 0x100000
 8006e78:	d10b      	bne.n	8006e92 <__aeabi_ddiv+0x11e>
 8006e7a:	ea41 0100 	orr.w	r1, r1, r0
 8006e7e:	f04f 0000 	mov.w	r0, #0
 8006e82:	f04f 4c00 	mov.w	ip, #2147483648	; 0x80000000
 8006e86:	e7b6      	b.n	8006df6 <__aeabi_ddiv+0x82>
 8006e88:	f411 1f80 	tst.w	r1, #1048576	; 0x100000
 8006e8c:	bf04      	itt	eq
 8006e8e:	4301      	orreq	r1, r0
 8006e90:	2000      	moveq	r0, #0
 8006e92:	f1b4 0cfd 	subs.w	ip, r4, #253	; 0xfd
 8006e96:	bf88      	it	hi
 8006e98:	f5bc 6fe0 	cmphi.w	ip, #1792	; 0x700
 8006e9c:	f63f aeaf 	bhi.w	8006bfe <__aeabi_dmul+0xde>
 8006ea0:	ebb5 0c03 	subs.w	ip, r5, r3
 8006ea4:	bf04      	itt	eq
 8006ea6:	ebb6 0c02 	subseq.w	ip, r6, r2
 8006eaa:	ea5f 0c50 	movseq.w	ip, r0, lsr #1
 8006eae:	f150 0000 	adcs.w	r0, r0, #0
 8006eb2:	eb41 5104 	adc.w	r1, r1, r4, lsl #20
 8006eb6:	bd70      	pop	{r4, r5, r6, pc}
 8006eb8:	f00e 4e00 	and.w	lr, lr, #2147483648	; 0x80000000
 8006ebc:	ea4e 3111 	orr.w	r1, lr, r1, lsr #12
 8006ec0:	eb14 045c 	adds.w	r4, r4, ip, lsr #1
 8006ec4:	bfc2      	ittt	gt
 8006ec6:	ebd4 050c 	rsbsgt	r5, r4, ip
 8006eca:	ea41 5104 	orrgt.w	r1, r1, r4, lsl #20
 8006ece:	bd70      	popgt	{r4, r5, r6, pc}
 8006ed0:	f441 1180 	orr.w	r1, r1, #1048576	; 0x100000
 8006ed4:	f04f 0e00 	mov.w	lr, #0
 8006ed8:	3c01      	subs	r4, #1
 8006eda:	e690      	b.n	8006bfe <__aeabi_dmul+0xde>
 8006edc:	ea45 0e06 	orr.w	lr, r5, r6
 8006ee0:	e68d      	b.n	8006bfe <__aeabi_dmul+0xde>
 8006ee2:	ea0c 5513 	and.w	r5, ip, r3, lsr #20
 8006ee6:	ea94 0f0c 	teq	r4, ip
 8006eea:	bf08      	it	eq
 8006eec:	ea95 0f0c 	teqeq	r5, ip
 8006ef0:	f43f af3b 	beq.w	8006d6a <__aeabi_dmul+0x24a>
 8006ef4:	ea94 0f0c 	teq	r4, ip
 8006ef8:	d10a      	bne.n	8006f10 <__aeabi_ddiv+0x19c>
 8006efa:	ea50 3401 	orrs.w	r4, r0, r1, lsl #12
 8006efe:	f47f af34 	bne.w	8006d6a <__aeabi_dmul+0x24a>
 8006f02:	ea95 0f0c 	teq	r5, ip
 8006f06:	f47f af25 	bne.w	8006d54 <__aeabi_dmul+0x234>
 8006f0a:	4610      	mov	r0, r2
 8006f0c:	4619      	mov	r1, r3
 8006f0e:	e72c      	b.n	8006d6a <__aeabi_dmul+0x24a>
 8006f10:	ea95 0f0c 	teq	r5, ip
 8006f14:	d106      	bne.n	8006f24 <__aeabi_ddiv+0x1b0>
 8006f16:	ea52 3503 	orrs.w	r5, r2, r3, lsl #12
 8006f1a:	f43f aefd 	beq.w	8006d18 <__aeabi_dmul+0x1f8>
 8006f1e:	4610      	mov	r0, r2
 8006f20:	4619      	mov	r1, r3
 8006f22:	e722      	b.n	8006d6a <__aeabi_dmul+0x24a>
 8006f24:	ea50 0641 	orrs.w	r6, r0, r1, lsl #1
 8006f28:	bf18      	it	ne
 8006f2a:	ea52 0643 	orrsne.w	r6, r2, r3, lsl #1
 8006f2e:	f47f aec5 	bne.w	8006cbc <__aeabi_dmul+0x19c>
 8006f32:	ea50 0441 	orrs.w	r4, r0, r1, lsl #1
 8006f36:	f47f af0d 	bne.w	8006d54 <__aeabi_dmul+0x234>
 8006f3a:	ea52 0543 	orrs.w	r5, r2, r3, lsl #1
 8006f3e:	f47f aeeb 	bne.w	8006d18 <__aeabi_dmul+0x1f8>
 8006f42:	e712      	b.n	8006d6a <__aeabi_dmul+0x24a>

08006f44 <__aeabi_d2f>:
 8006f44:	ea4f 0241 	mov.w	r2, r1, lsl #1
 8006f48:	f1b2 43e0 	subs.w	r3, r2, #1879048192	; 0x70000000
 8006f4c:	bf24      	itt	cs
 8006f4e:	f5b3 1c00 	subscs.w	ip, r3, #2097152	; 0x200000
 8006f52:	f1dc 5cfe 	rsbscs	ip, ip, #532676608	; 0x1fc00000
 8006f56:	d90d      	bls.n	8006f74 <__aeabi_d2f+0x30>
 8006f58:	f001 4c00 	and.w	ip, r1, #2147483648	; 0x80000000
 8006f5c:	ea4f 02c0 	mov.w	r2, r0, lsl #3
 8006f60:	ea4c 7050 	orr.w	r0, ip, r0, lsr #29
 8006f64:	f1b2 4f00 	cmp.w	r2, #2147483648	; 0x80000000
 8006f68:	eb40 0083 	adc.w	r0, r0, r3, lsl #2
 8006f6c:	bf08      	it	eq
 8006f6e:	f020 0001 	biceq.w	r0, r0, #1
 8006f72:	4770      	bx	lr
 8006f74:	f011 4f80 	tst.w	r1, #1073741824	; 0x40000000
 8006f78:	d121      	bne.n	8006fbe <__aeabi_d2f+0x7a>
 8006f7a:	f113 7238 	adds.w	r2, r3, #48234496	; 0x2e00000
 8006f7e:	bfbc      	itt	lt
 8006f80:	f001 4000 	andlt.w	r0, r1, #2147483648	; 0x80000000
 8006f84:	4770      	bxlt	lr
 8006f86:	f441 1180 	orr.w	r1, r1, #1048576	; 0x100000
 8006f8a:	ea4f 5252 	mov.w	r2, r2, lsr #21
 8006f8e:	f1c2 0218 	rsb	r2, r2, #24
 8006f92:	f1c2 0c20 	rsb	ip, r2, #32
 8006f96:	fa10 f30c 	lsls.w	r3, r0, ip
 8006f9a:	fa20 f002 	lsr.w	r0, r0, r2
 8006f9e:	bf18      	it	ne
 8006fa0:	f040 0001 	orrne.w	r0, r0, #1
 8006fa4:	ea4f 23c1 	mov.w	r3, r1, lsl #11
 8006fa8:	ea4f 23d3 	mov.w	r3, r3, lsr #11
 8006fac:	fa03 fc0c 	lsl.w	ip, r3, ip
 8006fb0:	ea40 000c 	orr.w	r0, r0, ip
 8006fb4:	fa23 f302 	lsr.w	r3, r3, r2
 8006fb8:	ea4f 0343 	mov.w	r3, r3, lsl #1
 8006fbc:	e7cc      	b.n	8006f58 <__aeabi_d2f+0x14>
 8006fbe:	ea7f 5362 	mvns.w	r3, r2, asr #21
 8006fc2:	d107      	bne.n	8006fd4 <__aeabi_d2f+0x90>
 8006fc4:	ea50 3301 	orrs.w	r3, r0, r1, lsl #12
 8006fc8:	bf1e      	ittt	ne
 8006fca:	f04f 40fe 	movne.w	r0, #2130706432	; 0x7f000000
 8006fce:	f440 0040 	orrne.w	r0, r0, #12582912	; 0xc00000
 8006fd2:	4770      	bxne	lr
 8006fd4:	f001 4000 	and.w	r0, r1, #2147483648	; 0x80000000
 8006fd8:	f040 40fe 	orr.w	r0, r0, #2130706432	; 0x7f000000
 8006fdc:	f440 0000 	orr.w	r0, r0, #8388608	; 0x800000
 8006fe0:	4770      	bx	lr
 8006fe2:	bf00      	nop

08006fe4 <__aeabi_frsub>:
 8006fe4:	f080 4000 	eor.w	r0, r0, #2147483648	; 0x80000000
 8006fe8:	e002      	b.n	8006ff0 <__addsf3>
 8006fea:	bf00      	nop

08006fec <__aeabi_fsub>:
 8006fec:	f081 4100 	eor.w	r1, r1, #2147483648	; 0x80000000

08006ff0 <__addsf3>:
 8006ff0:	0042      	lsls	r2, r0, #1
 8006ff2:	bf1f      	itttt	ne
 8006ff4:	ea5f 0341 	movsne.w	r3, r1, lsl #1
 8006ff8:	ea92 0f03 	teqne	r2, r3
 8006ffc:	ea7f 6c22 	mvnsne.w	ip, r2, asr #24
 8007000:	ea7f 6c23 	mvnsne.w	ip, r3, asr #24
 8007004:	d06a      	beq.n	80070dc <__addsf3+0xec>
 8007006:	ea4f 6212 	mov.w	r2, r2, lsr #24
 800700a:	ebd2 6313 	rsbs	r3, r2, r3, lsr #24
 800700e:	bfc1      	itttt	gt
 8007010:	18d2      	addgt	r2, r2, r3
 8007012:	4041      	eorgt	r1, r0
 8007014:	4048      	eorgt	r0, r1
 8007016:	4041      	eorgt	r1, r0
 8007018:	bfb8      	it	lt
 800701a:	425b      	neglt	r3, r3
 800701c:	2b19      	cmp	r3, #25
 800701e:	bf88      	it	hi
 8007020:	4770      	bxhi	lr
 8007022:	f010 4f00 	tst.w	r0, #2147483648	; 0x80000000
 8007026:	f440 0000 	orr.w	r0, r0, #8388608	; 0x800000
 800702a:	f020 407f 	bic.w	r0, r0, #4278190080	; 0xff000000
 800702e:	bf18      	it	ne
 8007030:	4240      	negne	r0, r0
 8007032:	f011 4f00 	tst.w	r1, #2147483648	; 0x80000000
 8007036:	f441 0100 	orr.w	r1, r1, #8388608	; 0x800000
 800703a:	f021 417f 	bic.w	r1, r1, #4278190080	; 0xff000000
 800703e:	bf18      	it	ne
 8007040:	4249      	negne	r1, r1
 8007042:	ea92 0f03 	teq	r2, r3
 8007046:	d03f      	beq.n	80070c8 <__addsf3+0xd8>
 8007048:	f1a2 0201 	sub.w	r2, r2, #1
 800704c:	fa41 fc03 	asr.w	ip, r1, r3
 8007050:	eb10 000c 	adds.w	r0, r0, ip
 8007054:	f1c3 0320 	rsb	r3, r3, #32
 8007058:	fa01 f103 	lsl.w	r1, r1, r3
 800705c:	f000 4300 	and.w	r3, r0, #2147483648	; 0x80000000
 8007060:	d502      	bpl.n	8007068 <__addsf3+0x78>
 8007062:	4249      	negs	r1, r1
 8007064:	eb60 0040 	sbc.w	r0, r0, r0, lsl #1
 8007068:	f5b0 0f00 	cmp.w	r0, #8388608	; 0x800000
 800706c:	d313      	bcc.n	8007096 <__addsf3+0xa6>
 800706e:	f1b0 7f80 	cmp.w	r0, #16777216	; 0x1000000
 8007072:	d306      	bcc.n	8007082 <__addsf3+0x92>
 8007074:	0840      	lsrs	r0, r0, #1
 8007076:	ea4f 0131 	mov.w	r1, r1, rrx
 800707a:	f102 0201 	add.w	r2, r2, #1
 800707e:	2afe      	cmp	r2, #254	; 0xfe
 8007080:	d251      	bcs.n	8007126 <__addsf3+0x136>
 8007082:	f1b1 4f00 	cmp.w	r1, #2147483648	; 0x80000000
 8007086:	eb40 50c2 	adc.w	r0, r0, r2, lsl #23
 800708a:	bf08      	it	eq
 800708c:	f020 0001 	biceq.w	r0, r0, #1
 8007090:	ea40 0003 	orr.w	r0, r0, r3
 8007094:	4770      	bx	lr
 8007096:	0049      	lsls	r1, r1, #1
 8007098:	eb40 0000 	adc.w	r0, r0, r0
 800709c:	f410 0f00 	tst.w	r0, #8388608	; 0x800000
 80070a0:	f1a2 0201 	sub.w	r2, r2, #1
 80070a4:	d1ed      	bne.n	8007082 <__addsf3+0x92>
 80070a6:	fab0 fc80 	clz	ip, r0
 80070aa:	f1ac 0c08 	sub.w	ip, ip, #8
 80070ae:	ebb2 020c 	subs.w	r2, r2, ip
 80070b2:	fa00 f00c 	lsl.w	r0, r0, ip
 80070b6:	bfaa      	itet	ge
 80070b8:	eb00 50c2 	addge.w	r0, r0, r2, lsl #23
 80070bc:	4252      	neglt	r2, r2
 80070be:	4318      	orrge	r0, r3
 80070c0:	bfbc      	itt	lt
 80070c2:	40d0      	lsrlt	r0, r2
 80070c4:	4318      	orrlt	r0, r3
 80070c6:	4770      	bx	lr
 80070c8:	f092 0f00 	teq	r2, #0
 80070cc:	f481 0100 	eor.w	r1, r1, #8388608	; 0x800000
 80070d0:	bf06      	itte	eq
 80070d2:	f480 0000 	eoreq.w	r0, r0, #8388608	; 0x800000
 80070d6:	3201      	addeq	r2, #1
 80070d8:	3b01      	subne	r3, #1
 80070da:	e7b5      	b.n	8007048 <__addsf3+0x58>
 80070dc:	ea4f 0341 	mov.w	r3, r1, lsl #1
 80070e0:	ea7f 6c22 	mvns.w	ip, r2, asr #24
 80070e4:	bf18      	it	ne
 80070e6:	ea7f 6c23 	mvnsne.w	ip, r3, asr #24
 80070ea:	d021      	beq.n	8007130 <__addsf3+0x140>
 80070ec:	ea92 0f03 	teq	r2, r3
 80070f0:	d004      	beq.n	80070fc <__addsf3+0x10c>
 80070f2:	f092 0f00 	teq	r2, #0
 80070f6:	bf08      	it	eq
 80070f8:	4608      	moveq	r0, r1
 80070fa:	4770      	bx	lr
 80070fc:	ea90 0f01 	teq	r0, r1
 8007100:	bf1c      	itt	ne
 8007102:	2000      	movne	r0, #0
 8007104:	4770      	bxne	lr
 8007106:	f012 4f7f 	tst.w	r2, #4278190080	; 0xff000000
 800710a:	d104      	bne.n	8007116 <__addsf3+0x126>
 800710c:	0040      	lsls	r0, r0, #1
 800710e:	bf28      	it	cs
 8007110:	f040 4000 	orrcs.w	r0, r0, #2147483648	; 0x80000000
 8007114:	4770      	bx	lr
 8007116:	f112 7200 	adds.w	r2, r2, #33554432	; 0x2000000
 800711a:	bf3c      	itt	cc
 800711c:	f500 0000 	addcc.w	r0, r0, #8388608	; 0x800000
 8007120:	4770      	bxcc	lr
 8007122:	f000 4300 	and.w	r3, r0, #2147483648	; 0x80000000
 8007126:	f043 40fe 	orr.w	r0, r3, #2130706432	; 0x7f000000
 800712a:	f440 0000 	orr.w	r0, r0, #8388608	; 0x800000
 800712e:	4770      	bx	lr
 8007130:	ea7f 6222 	mvns.w	r2, r2, asr #24
 8007134:	bf16      	itet	ne
 8007136:	4608      	movne	r0, r1
 8007138:	ea7f 6323 	mvnseq.w	r3, r3, asr #24
 800713c:	4601      	movne	r1, r0
 800713e:	0242      	lsls	r2, r0, #9
 8007140:	bf06      	itte	eq
 8007142:	ea5f 2341 	movseq.w	r3, r1, lsl #9
 8007146:	ea90 0f01 	teqeq	r0, r1
 800714a:	f440 0080 	orrne.w	r0, r0, #4194304	; 0x400000
 800714e:	4770      	bx	lr

08007150 <__aeabi_ui2f>:
 8007150:	f04f 0300 	mov.w	r3, #0
 8007154:	e004      	b.n	8007160 <__aeabi_i2f+0x8>
 8007156:	bf00      	nop

08007158 <__aeabi_i2f>:
 8007158:	f010 4300 	ands.w	r3, r0, #2147483648	; 0x80000000
 800715c:	bf48      	it	mi
 800715e:	4240      	negmi	r0, r0
 8007160:	ea5f 0c00 	movs.w	ip, r0
 8007164:	bf08      	it	eq
 8007166:	4770      	bxeq	lr
 8007168:	f043 4396 	orr.w	r3, r3, #1258291200	; 0x4b000000
 800716c:	4601      	mov	r1, r0
 800716e:	f04f 0000 	mov.w	r0, #0
 8007172:	e01c      	b.n	80071ae <__aeabi_l2f+0x2a>

08007174 <__aeabi_ul2f>:
 8007174:	ea50 0201 	orrs.w	r2, r0, r1
 8007178:	bf08      	it	eq
 800717a:	4770      	bxeq	lr
 800717c:	f04f 0300 	mov.w	r3, #0
 8007180:	e00a      	b.n	8007198 <__aeabi_l2f+0x14>
 8007182:	bf00      	nop

08007184 <__aeabi_l2f>:
 8007184:	ea50 0201 	orrs.w	r2, r0, r1
 8007188:	bf08      	it	eq
 800718a:	4770      	bxeq	lr
 800718c:	f011 4300 	ands.w	r3, r1, #2147483648	; 0x80000000
 8007190:	d502      	bpl.n	8007198 <__aeabi_l2f+0x14>
 8007192:	4240      	negs	r0, r0
 8007194:	eb61 0141 	sbc.w	r1, r1, r1, lsl #1
 8007198:	ea5f 0c01 	movs.w	ip, r1
 800719c:	bf02      	ittt	eq
 800719e:	4684      	moveq	ip, r0
 80071a0:	4601      	moveq	r1, r0
 80071a2:	2000      	moveq	r0, #0
 80071a4:	f043 43b6 	orr.w	r3, r3, #1526726656	; 0x5b000000
 80071a8:	bf08      	it	eq
 80071aa:	f1a3 5380 	subeq.w	r3, r3, #268435456	; 0x10000000
 80071ae:	f5a3 0300 	sub.w	r3, r3, #8388608	; 0x800000
 80071b2:	fabc f28c 	clz	r2, ip
 80071b6:	3a08      	subs	r2, #8
 80071b8:	eba3 53c2 	sub.w	r3, r3, r2, lsl #23
 80071bc:	db10      	blt.n	80071e0 <__aeabi_l2f+0x5c>
 80071be:	fa01 fc02 	lsl.w	ip, r1, r2
 80071c2:	4463      	add	r3, ip
 80071c4:	fa00 fc02 	lsl.w	ip, r0, r2
 80071c8:	f1c2 0220 	rsb	r2, r2, #32
 80071cc:	f1bc 4f00 	cmp.w	ip, #2147483648	; 0x80000000
 80071d0:	fa20 f202 	lsr.w	r2, r0, r2
 80071d4:	eb43 0002 	adc.w	r0, r3, r2
 80071d8:	bf08      	it	eq
 80071da:	f020 0001 	biceq.w	r0, r0, #1
 80071de:	4770      	bx	lr
 80071e0:	f102 0220 	add.w	r2, r2, #32
 80071e4:	fa01 fc02 	lsl.w	ip, r1, r2
 80071e8:	f1c2 0220 	rsb	r2, r2, #32
 80071ec:	ea50 004c 	orrs.w	r0, r0, ip, lsl #1
 80071f0:	fa21 f202 	lsr.w	r2, r1, r2
 80071f4:	eb43 0002 	adc.w	r0, r3, r2
 80071f8:	bf08      	it	eq
 80071fa:	ea20 70dc 	biceq.w	r0, r0, ip, lsr #31
 80071fe:	4770      	bx	lr

08007200 <__aeabi_fmul>:
 8007200:	f04f 0cff 	mov.w	ip, #255	; 0xff
 8007204:	ea1c 52d0 	ands.w	r2, ip, r0, lsr #23
 8007208:	bf1e      	ittt	ne
 800720a:	ea1c 53d1 	andsne.w	r3, ip, r1, lsr #23
 800720e:	ea92 0f0c 	teqne	r2, ip
 8007212:	ea93 0f0c 	teqne	r3, ip
 8007216:	d06f      	beq.n	80072f8 <__aeabi_fmul+0xf8>
 8007218:	441a      	add	r2, r3
 800721a:	ea80 0c01 	eor.w	ip, r0, r1
 800721e:	0240      	lsls	r0, r0, #9
 8007220:	bf18      	it	ne
 8007222:	ea5f 2141 	movsne.w	r1, r1, lsl #9
 8007226:	d01e      	beq.n	8007266 <__aeabi_fmul+0x66>
 8007228:	f04f 6300 	mov.w	r3, #134217728	; 0x8000000
 800722c:	ea43 1050 	orr.w	r0, r3, r0, lsr #5
 8007230:	ea43 1151 	orr.w	r1, r3, r1, lsr #5
 8007234:	fba0 3101 	umull	r3, r1, r0, r1
 8007238:	f00c 4000 	and.w	r0, ip, #2147483648	; 0x80000000
 800723c:	f5b1 0f00 	cmp.w	r1, #8388608	; 0x800000
 8007240:	bf3e      	ittt	cc
 8007242:	0049      	lslcc	r1, r1, #1
 8007244:	ea41 71d3 	orrcc.w	r1, r1, r3, lsr #31
 8007248:	005b      	lslcc	r3, r3, #1
 800724a:	ea40 0001 	orr.w	r0, r0, r1
 800724e:	f162 027f 	sbc.w	r2, r2, #127	; 0x7f
 8007252:	2afd      	cmp	r2, #253	; 0xfd
 8007254:	d81d      	bhi.n	8007292 <__aeabi_fmul+0x92>
 8007256:	f1b3 4f00 	cmp.w	r3, #2147483648	; 0x80000000
 800725a:	eb40 50c2 	adc.w	r0, r0, r2, lsl #23
 800725e:	bf08      	it	eq
 8007260:	f020 0001 	biceq.w	r0, r0, #1
 8007264:	4770      	bx	lr
 8007266:	f090 0f00 	teq	r0, #0
 800726a:	f00c 4c00 	and.w	ip, ip, #2147483648	; 0x80000000
 800726e:	bf08      	it	eq
 8007270:	0249      	lsleq	r1, r1, #9
 8007272:	ea4c 2050 	orr.w	r0, ip, r0, lsr #9
 8007276:	ea40 2051 	orr.w	r0, r0, r1, lsr #9
 800727a:	3a7f      	subs	r2, #127	; 0x7f
 800727c:	bfc2      	ittt	gt
 800727e:	f1d2 03ff 	rsbsgt	r3, r2, #255	; 0xff
 8007282:	ea40 50c2 	orrgt.w	r0, r0, r2, lsl #23
 8007286:	4770      	bxgt	lr
 8007288:	f440 0000 	orr.w	r0, r0, #8388608	; 0x800000
 800728c:	f04f 0300 	mov.w	r3, #0
 8007290:	3a01      	subs	r2, #1
 8007292:	dc5d      	bgt.n	8007350 <__aeabi_fmul+0x150>
 8007294:	f112 0f19 	cmn.w	r2, #25
 8007298:	bfdc      	itt	le
 800729a:	f000 4000 	andle.w	r0, r0, #2147483648	; 0x80000000
 800729e:	4770      	bxle	lr
 80072a0:	f1c2 0200 	rsb	r2, r2, #0
 80072a4:	0041      	lsls	r1, r0, #1
 80072a6:	fa21 f102 	lsr.w	r1, r1, r2
 80072aa:	f1c2 0220 	rsb	r2, r2, #32
 80072ae:	fa00 fc02 	lsl.w	ip, r0, r2
 80072b2:	ea5f 0031 	movs.w	r0, r1, rrx
 80072b6:	f140 0000 	adc.w	r0, r0, #0
 80072ba:	ea53 034c 	orrs.w	r3, r3, ip, lsl #1
 80072be:	bf08      	it	eq
 80072c0:	ea20 70dc 	biceq.w	r0, r0, ip, lsr #31
 80072c4:	4770      	bx	lr
 80072c6:	f092 0f00 	teq	r2, #0
 80072ca:	f000 4c00 	and.w	ip, r0, #2147483648	; 0x80000000
 80072ce:	bf02      	ittt	eq
 80072d0:	0040      	lsleq	r0, r0, #1
 80072d2:	f410 0f00 	tsteq.w	r0, #8388608	; 0x800000
 80072d6:	3a01      	subeq	r2, #1
 80072d8:	d0f9      	beq.n	80072ce <__aeabi_fmul+0xce>
 80072da:	ea40 000c 	orr.w	r0, r0, ip
 80072de:	f093 0f00 	teq	r3, #0
 80072e2:	f001 4c00 	and.w	ip, r1, #2147483648	; 0x80000000
 80072e6:	bf02      	ittt	eq
 80072e8:	0049      	lsleq	r1, r1, #1
 80072ea:	f411 0f00 	tsteq.w	r1, #8388608	; 0x800000
 80072ee:	3b01      	subeq	r3, #1
 80072f0:	d0f9      	beq.n	80072e6 <__aeabi_fmul+0xe6>
 80072f2:	ea41 010c 	orr.w	r1, r1, ip
 80072f6:	e78f      	b.n	8007218 <__aeabi_fmul+0x18>
 80072f8:	ea0c 53d1 	and.w	r3, ip, r1, lsr #23
 80072fc:	ea92 0f0c 	teq	r2, ip
 8007300:	bf18      	it	ne
 8007302:	ea93 0f0c 	teqne	r3, ip
 8007306:	d00a      	beq.n	800731e <__aeabi_fmul+0x11e>
 8007308:	f030 4c00 	bics.w	ip, r0, #2147483648	; 0x80000000
 800730c:	bf18      	it	ne
 800730e:	f031 4c00 	bicsne.w	ip, r1, #2147483648	; 0x80000000
 8007312:	d1d8      	bne.n	80072c6 <__aeabi_fmul+0xc6>
 8007314:	ea80 0001 	eor.w	r0, r0, r1
 8007318:	f000 4000 	and.w	r0, r0, #2147483648	; 0x80000000
 800731c:	4770      	bx	lr
 800731e:	f090 0f00 	teq	r0, #0
 8007322:	bf17      	itett	ne
 8007324:	f090 4f00 	teqne	r0, #2147483648	; 0x80000000
 8007328:	4608      	moveq	r0, r1
 800732a:	f091 0f00 	teqne	r1, #0
 800732e:	f091 4f00 	teqne	r1, #2147483648	; 0x80000000
 8007332:	d014      	beq.n	800735e <__aeabi_fmul+0x15e>
 8007334:	ea92 0f0c 	teq	r2, ip
 8007338:	d101      	bne.n	800733e <__aeabi_fmul+0x13e>
 800733a:	0242      	lsls	r2, r0, #9
 800733c:	d10f      	bne.n	800735e <__aeabi_fmul+0x15e>
 800733e:	ea93 0f0c 	teq	r3, ip
 8007342:	d103      	bne.n	800734c <__aeabi_fmul+0x14c>
 8007344:	024b      	lsls	r3, r1, #9
 8007346:	bf18      	it	ne
 8007348:	4608      	movne	r0, r1
 800734a:	d108      	bne.n	800735e <__aeabi_fmul+0x15e>
 800734c:	ea80 0001 	eor.w	r0, r0, r1
 8007350:	f000 4000 	and.w	r0, r0, #2147483648	; 0x80000000
 8007354:	f040 40fe 	orr.w	r0, r0, #2130706432	; 0x7f000000
 8007358:	f440 0000 	orr.w	r0, r0, #8388608	; 0x800000
 800735c:	4770      	bx	lr
 800735e:	f040 40fe 	orr.w	r0, r0, #2130706432	; 0x7f000000
 8007362:	f440 0040 	orr.w	r0, r0, #12582912	; 0xc00000
 8007366:	4770      	bx	lr

08007368 <__aeabi_fdiv>:
 8007368:	f04f 0cff 	mov.w	ip, #255	; 0xff
 800736c:	ea1c 52d0 	ands.w	r2, ip, r0, lsr #23
 8007370:	bf1e      	ittt	ne
 8007372:	ea1c 53d1 	andsne.w	r3, ip, r1, lsr #23
 8007376:	ea92 0f0c 	teqne	r2, ip
 800737a:	ea93 0f0c 	teqne	r3, ip
 800737e:	d069      	beq.n	8007454 <__aeabi_fdiv+0xec>
 8007380:	eba2 0203 	sub.w	r2, r2, r3
 8007384:	ea80 0c01 	eor.w	ip, r0, r1
 8007388:	0249      	lsls	r1, r1, #9
 800738a:	ea4f 2040 	mov.w	r0, r0, lsl #9
 800738e:	d037      	beq.n	8007400 <__aeabi_fdiv+0x98>
 8007390:	f04f 5380 	mov.w	r3, #268435456	; 0x10000000
 8007394:	ea43 1111 	orr.w	r1, r3, r1, lsr #4
 8007398:	ea43 1310 	orr.w	r3, r3, r0, lsr #4
 800739c:	f00c 4000 	and.w	r0, ip, #2147483648	; 0x80000000
 80073a0:	428b      	cmp	r3, r1
 80073a2:	bf38      	it	cc
 80073a4:	005b      	lslcc	r3, r3, #1
 80073a6:	f142 027d 	adc.w	r2, r2, #125	; 0x7d
 80073aa:	f44f 0c00 	mov.w	ip, #8388608	; 0x800000
 80073ae:	428b      	cmp	r3, r1
 80073b0:	bf24      	itt	cs
 80073b2:	1a5b      	subcs	r3, r3, r1
 80073b4:	ea40 000c 	orrcs.w	r0, r0, ip
 80073b8:	ebb3 0f51 	cmp.w	r3, r1, lsr #1
 80073bc:	bf24      	itt	cs
 80073be:	eba3 0351 	subcs.w	r3, r3, r1, lsr #1
 80073c2:	ea40 005c 	orrcs.w	r0, r0, ip, lsr #1
 80073c6:	ebb3 0f91 	cmp.w	r3, r1, lsr #2
 80073ca:	bf24      	itt	cs
 80073cc:	eba3 0391 	subcs.w	r3, r3, r1, lsr #2
 80073d0:	ea40 009c 	orrcs.w	r0, r0, ip, lsr #2
 80073d4:	ebb3 0fd1 	cmp.w	r3, r1, lsr #3
 80073d8:	bf24      	itt	cs
 80073da:	eba3 03d1 	subcs.w	r3, r3, r1, lsr #3
 80073de:	ea40 00dc 	orrcs.w	r0, r0, ip, lsr #3
 80073e2:	011b      	lsls	r3, r3, #4
 80073e4:	bf18      	it	ne
 80073e6:	ea5f 1c1c 	movsne.w	ip, ip, lsr #4
 80073ea:	d1e0      	bne.n	80073ae <__aeabi_fdiv+0x46>
 80073ec:	2afd      	cmp	r2, #253	; 0xfd
 80073ee:	f63f af50 	bhi.w	8007292 <__aeabi_fmul+0x92>
 80073f2:	428b      	cmp	r3, r1
 80073f4:	eb40 50c2 	adc.w	r0, r0, r2, lsl #23
 80073f8:	bf08      	it	eq
 80073fa:	f020 0001 	biceq.w	r0, r0, #1
 80073fe:	4770      	bx	lr
 8007400:	f00c 4c00 	and.w	ip, ip, #2147483648	; 0x80000000
 8007404:	ea4c 2050 	orr.w	r0, ip, r0, lsr #9
 8007408:	327f      	adds	r2, #127	; 0x7f
 800740a:	bfc2      	ittt	gt
 800740c:	f1d2 03ff 	rsbsgt	r3, r2, #255	; 0xff
 8007410:	ea40 50c2 	orrgt.w	r0, r0, r2, lsl #23
 8007414:	4770      	bxgt	lr
 8007416:	f440 0000 	orr.w	r0, r0, #8388608	; 0x800000
 800741a:	f04f 0300 	mov.w	r3, #0
 800741e:	3a01      	subs	r2, #1
 8007420:	e737      	b.n	8007292 <__aeabi_fmul+0x92>
 8007422:	f092 0f00 	teq	r2, #0
 8007426:	f000 4c00 	and.w	ip, r0, #2147483648	; 0x80000000
 800742a:	bf02      	ittt	eq
 800742c:	0040      	lsleq	r0, r0, #1
 800742e:	f410 0f00 	tsteq.w	r0, #8388608	; 0x800000
 8007432:	3a01      	subeq	r2, #1
 8007434:	d0f9      	beq.n	800742a <__aeabi_fdiv+0xc2>
 8007436:	ea40 000c 	orr.w	r0, r0, ip
 800743a:	f093 0f00 	teq	r3, #0
 800743e:	f001 4c00 	and.w	ip, r1, #2147483648	; 0x80000000
 8007442:	bf02      	ittt	eq
 8007444:	0049      	lsleq	r1, r1, #1
 8007446:	f411 0f00 	tsteq.w	r1, #8388608	; 0x800000
 800744a:	3b01      	subeq	r3, #1
 800744c:	d0f9      	beq.n	8007442 <__aeabi_fdiv+0xda>
 800744e:	ea41 010c 	orr.w	r1, r1, ip
 8007452:	e795      	b.n	8007380 <__aeabi_fdiv+0x18>
 8007454:	ea0c 53d1 	and.w	r3, ip, r1, lsr #23
 8007458:	ea92 0f0c 	teq	r2, ip
 800745c:	d108      	bne.n	8007470 <__aeabi_fdiv+0x108>
 800745e:	0242      	lsls	r2, r0, #9
 8007460:	f47f af7d 	bne.w	800735e <__aeabi_fmul+0x15e>
 8007464:	ea93 0f0c 	teq	r3, ip
 8007468:	f47f af70 	bne.w	800734c <__aeabi_fmul+0x14c>
 800746c:	4608      	mov	r0, r1
 800746e:	e776      	b.n	800735e <__aeabi_fmul+0x15e>
 8007470:	ea93 0f0c 	teq	r3, ip
 8007474:	d104      	bne.n	8007480 <__aeabi_fdiv+0x118>
 8007476:	024b      	lsls	r3, r1, #9
 8007478:	f43f af4c 	beq.w	8007314 <__aeabi_fmul+0x114>
 800747c:	4608      	mov	r0, r1
 800747e:	e76e      	b.n	800735e <__aeabi_fmul+0x15e>
 8007480:	f030 4c00 	bics.w	ip, r0, #2147483648	; 0x80000000
 8007484:	bf18      	it	ne
 8007486:	f031 4c00 	bicsne.w	ip, r1, #2147483648	; 0x80000000
 800748a:	d1ca      	bne.n	8007422 <__aeabi_fdiv+0xba>
 800748c:	f030 4200 	bics.w	r2, r0, #2147483648	; 0x80000000
 8007490:	f47f af5c 	bne.w	800734c <__aeabi_fmul+0x14c>
 8007494:	f031 4300 	bics.w	r3, r1, #2147483648	; 0x80000000
 8007498:	f47f af3c 	bne.w	8007314 <__aeabi_fmul+0x114>
 800749c:	e75f      	b.n	800735e <__aeabi_fmul+0x15e>
 800749e:	bf00      	nop

080074a0 <__gesf2>:
 80074a0:	f04f 3cff 	mov.w	ip, #4294967295
 80074a4:	e006      	b.n	80074b4 <__cmpsf2+0x4>
 80074a6:	bf00      	nop

080074a8 <__lesf2>:
 80074a8:	f04f 0c01 	mov.w	ip, #1
 80074ac:	e002      	b.n	80074b4 <__cmpsf2+0x4>
 80074ae:	bf00      	nop

080074b0 <__cmpsf2>:
 80074b0:	f04f 0c01 	mov.w	ip, #1
 80074b4:	f84d cd04 	str.w	ip, [sp, #-4]!
 80074b8:	ea4f 0240 	mov.w	r2, r0, lsl #1
 80074bc:	ea4f 0341 	mov.w	r3, r1, lsl #1
 80074c0:	ea7f 6c22 	mvns.w	ip, r2, asr #24
 80074c4:	bf18      	it	ne
 80074c6:	ea7f 6c23 	mvnsne.w	ip, r3, asr #24
 80074ca:	d011      	beq.n	80074f0 <__cmpsf2+0x40>
 80074cc:	b001      	add	sp, #4
 80074ce:	ea52 0c53 	orrs.w	ip, r2, r3, lsr #1
 80074d2:	bf18      	it	ne
 80074d4:	ea90 0f01 	teqne	r0, r1
 80074d8:	bf58      	it	pl
 80074da:	ebb2 0003 	subspl.w	r0, r2, r3
 80074de:	bf88      	it	hi
 80074e0:	17c8      	asrhi	r0, r1, #31
 80074e2:	bf38      	it	cc
 80074e4:	ea6f 70e1 	mvncc.w	r0, r1, asr #31
 80074e8:	bf18      	it	ne
 80074ea:	f040 0001 	orrne.w	r0, r0, #1
 80074ee:	4770      	bx	lr
 80074f0:	ea7f 6c22 	mvns.w	ip, r2, asr #24
 80074f4:	d102      	bne.n	80074fc <__cmpsf2+0x4c>
 80074f6:	ea5f 2c40 	movs.w	ip, r0, lsl #9
 80074fa:	d105      	bne.n	8007508 <__cmpsf2+0x58>
 80074fc:	ea7f 6c23 	mvns.w	ip, r3, asr #24
 8007500:	d1e4      	bne.n	80074cc <__cmpsf2+0x1c>
 8007502:	ea5f 2c41 	movs.w	ip, r1, lsl #9
 8007506:	d0e1      	beq.n	80074cc <__cmpsf2+0x1c>
 8007508:	f85d 0b04 	ldr.w	r0, [sp], #4
 800750c:	4770      	bx	lr
 800750e:	bf00      	nop

08007510 <__aeabi_cfrcmple>:
 8007510:	4684      	mov	ip, r0
 8007512:	4608      	mov	r0, r1
 8007514:	4661      	mov	r1, ip
 8007516:	e7ff      	b.n	8007518 <__aeabi_cfcmpeq>

08007518 <__aeabi_cfcmpeq>:
 8007518:	b50f      	push	{r0, r1, r2, r3, lr}
 800751a:	f7ff ffc9 	bl	80074b0 <__cmpsf2>
 800751e:	2800      	cmp	r0, #0
 8007520:	bf48      	it	mi
 8007522:	f110 0f00 	cmnmi.w	r0, #0
 8007526:	bd0f      	pop	{r0, r1, r2, r3, pc}

08007528 <__aeabi_fcmpeq>:
 8007528:	f84d ed08 	str.w	lr, [sp, #-8]!
 800752c:	f7ff fff4 	bl	8007518 <__aeabi_cfcmpeq>
 8007530:	bf0c      	ite	eq
 8007532:	2001      	moveq	r0, #1
 8007534:	2000      	movne	r0, #0
 8007536:	f85d fb08 	ldr.w	pc, [sp], #8
 800753a:	bf00      	nop

0800753c <__aeabi_fcmplt>:
 800753c:	f84d ed08 	str.w	lr, [sp, #-8]!
 8007540:	f7ff ffea 	bl	8007518 <__aeabi_cfcmpeq>
 8007544:	bf34      	ite	cc
 8007546:	2001      	movcc	r0, #1
 8007548:	2000      	movcs	r0, #0
 800754a:	f85d fb08 	ldr.w	pc, [sp], #8
 800754e:	bf00      	nop

08007550 <__aeabi_fcmple>:
 8007550:	f84d ed08 	str.w	lr, [sp, #-8]!
 8007554:	f7ff ffe0 	bl	8007518 <__aeabi_cfcmpeq>
 8007558:	bf94      	ite	ls
 800755a:	2001      	movls	r0, #1
 800755c:	2000      	movhi	r0, #0
 800755e:	f85d fb08 	ldr.w	pc, [sp], #8
 8007562:	bf00      	nop

08007564 <__aeabi_fcmpge>:
 8007564:	f84d ed08 	str.w	lr, [sp, #-8]!
 8007568:	f7ff ffd2 	bl	8007510 <__aeabi_cfrcmple>
 800756c:	bf94      	ite	ls
 800756e:	2001      	movls	r0, #1
 8007570:	2000      	movhi	r0, #0
 8007572:	f85d fb08 	ldr.w	pc, [sp], #8
 8007576:	bf00      	nop

08007578 <__aeabi_fcmpgt>:
 8007578:	f84d ed08 	str.w	lr, [sp, #-8]!
 800757c:	f7ff ffc8 	bl	8007510 <__aeabi_cfrcmple>
 8007580:	bf34      	ite	cc
 8007582:	2001      	movcc	r0, #1
 8007584:	2000      	movcs	r0, #0
 8007586:	f85d fb08 	ldr.w	pc, [sp], #8
 800758a:	bf00      	nop

0800758c <__aeabi_f2uiz>:
 800758c:	0042      	lsls	r2, r0, #1
 800758e:	d20e      	bcs.n	80075ae <__aeabi_f2uiz+0x22>
 8007590:	f1b2 4ffe 	cmp.w	r2, #2130706432	; 0x7f000000
 8007594:	d30b      	bcc.n	80075ae <__aeabi_f2uiz+0x22>
 8007596:	f04f 039e 	mov.w	r3, #158	; 0x9e
 800759a:	ebb3 6212 	subs.w	r2, r3, r2, lsr #24
 800759e:	d409      	bmi.n	80075b4 <__aeabi_f2uiz+0x28>
 80075a0:	ea4f 2300 	mov.w	r3, r0, lsl #8
 80075a4:	f043 4300 	orr.w	r3, r3, #2147483648	; 0x80000000
 80075a8:	fa23 f002 	lsr.w	r0, r3, r2
 80075ac:	4770      	bx	lr
 80075ae:	f04f 0000 	mov.w	r0, #0
 80075b2:	4770      	bx	lr
 80075b4:	f112 0f61 	cmn.w	r2, #97	; 0x61
 80075b8:	d101      	bne.n	80075be <__aeabi_f2uiz+0x32>
 80075ba:	0242      	lsls	r2, r0, #9
 80075bc:	d102      	bne.n	80075c4 <__aeabi_f2uiz+0x38>
 80075be:	f04f 30ff 	mov.w	r0, #4294967295
 80075c2:	4770      	bx	lr
 80075c4:	f04f 0000 	mov.w	r0, #0
 80075c8:	4770      	bx	lr
 80075ca:	bf00      	nop

080075cc <__cs3_premain>:
 80075cc:	b508      	push	{r3, lr}
 80075ce:	f000 f847 	bl	8007660 <__libc_init_array>
 80075d2:	4807      	ldr	r0, [pc, #28]	; (80075f0 <__cs3_premain+0x24>)
 80075d4:	b100      	cbz	r0, 80075d8 <__cs3_premain+0xc>
 80075d6:	6800      	ldr	r0, [r0, #0]
 80075d8:	4906      	ldr	r1, [pc, #24]	; (80075f4 <__cs3_premain+0x28>)
 80075da:	b101      	cbz	r1, 80075de <__cs3_premain+0x12>
 80075dc:	6809      	ldr	r1, [r1, #0]
 80075de:	2200      	movs	r2, #0
 80075e0:	f7fc fb5a 	bl	8003c98 <main>
 80075e4:	4b04      	ldr	r3, [pc, #16]	; (80075f8 <__cs3_premain+0x2c>)
 80075e6:	b10b      	cbz	r3, 80075ec <__cs3_premain+0x20>
 80075e8:	f3af 8000 	nop.w
 80075ec:	e7fe      	b.n	80075ec <__cs3_premain+0x20>
 80075ee:	bf00      	nop
	...

080075fc <__cs3_start_c>:
 80075fc:	b580      	push	{r7, lr}
 80075fe:	4911      	ldr	r1, [pc, #68]	; (8007644 <__cs3_start_c+0x48>)
 8007600:	4a11      	ldr	r2, [pc, #68]	; (8007648 <__cs3_start_c+0x4c>)
 8007602:	e01a      	b.n	800763a <__cs3_start_c+0x3e>
 8007604:	6855      	ldr	r5, [r2, #4]
 8007606:	6893      	ldr	r3, [r2, #8]
 8007608:	68d4      	ldr	r4, [r2, #12]
 800760a:	429d      	cmp	r5, r3
 800760c:	d009      	beq.n	8007622 <__cs3_start_c+0x26>
 800760e:	2000      	movs	r0, #0
 8007610:	e004      	b.n	800761c <__cs3_start_c+0x20>
 8007612:	e8f5 6702 	ldrd	r6, r7, [r5], #8
 8007616:	3008      	adds	r0, #8
 8007618:	e8e3 6702 	strd	r6, r7, [r3], #8
 800761c:	42a0      	cmp	r0, r4
 800761e:	d1f8      	bne.n	8007612 <__cs3_start_c+0x16>
 8007620:	e000      	b.n	8007624 <__cs3_start_c+0x28>
 8007622:	191b      	adds	r3, r3, r4
 8007624:	6914      	ldr	r4, [r2, #16]
 8007626:	2000      	movs	r0, #0
 8007628:	e004      	b.n	8007634 <__cs3_start_c+0x38>
 800762a:	3008      	adds	r0, #8
 800762c:	2600      	movs	r6, #0
 800762e:	2700      	movs	r7, #0
 8007630:	e8e3 6702 	strd	r6, r7, [r3], #8
 8007634:	42a0      	cmp	r0, r4
 8007636:	d1f8      	bne.n	800762a <__cs3_start_c+0x2e>
 8007638:	3214      	adds	r2, #20
 800763a:	3901      	subs	r1, #1
 800763c:	d2e2      	bcs.n	8007604 <__cs3_start_c+0x8>
 800763e:	f7ff ffc5 	bl	80075cc <__cs3_premain>
 8007642:	bf00      	nop
 8007644:	00000001 	.word	0x00000001
 8007648:	08007794 	.word	0x08007794

0800764c <__cs3_reset>:
 800764c:	4901      	ldr	r1, [pc, #4]	; (8007654 <__cs3_reset+0x8>)
 800764e:	468d      	mov	sp, r1
 8007650:	4901      	ldr	r1, [pc, #4]	; (8007658 <__cs3_reset+0xc>)
 8007652:	4708      	bx	r1
 8007654:	20005000 	.word	0x20005000
 8007658:	080075fd 	.word	0x080075fd

0800765c <__STM32DefaultExceptionHandler>:
 800765c:	e7fe      	b.n	800765c <__STM32DefaultExceptionHandler>
	...

08007660 <__libc_init_array>:
 8007660:	b570      	push	{r4, r5, r6, lr}
 8007662:	f247 7680 	movw	r6, #30592	; 0x7780
 8007666:	f247 7580 	movw	r5, #30592	; 0x7780
 800766a:	f6c0 0600 	movt	r6, #2048	; 0x800
 800766e:	f6c0 0500 	movt	r5, #2048	; 0x800
 8007672:	1b76      	subs	r6, r6, r5
 8007674:	10b6      	asrs	r6, r6, #2
 8007676:	d006      	beq.n	8007686 <__libc_init_array+0x26>
 8007678:	2400      	movs	r4, #0
 800767a:	f855 3b04 	ldr.w	r3, [r5], #4
 800767e:	3401      	adds	r4, #1
 8007680:	4798      	blx	r3
 8007682:	42a6      	cmp	r6, r4
 8007684:	d1f9      	bne.n	800767a <__libc_init_array+0x1a>
 8007686:	f247 7684 	movw	r6, #30596	; 0x7784
 800768a:	f247 7580 	movw	r5, #30592	; 0x7780
 800768e:	f6c0 0600 	movt	r6, #2048	; 0x800
 8007692:	f6c0 0500 	movt	r5, #2048	; 0x800
 8007696:	1b76      	subs	r6, r6, r5
 8007698:	f000 f86c 	bl	8007774 <_init>
 800769c:	10b6      	asrs	r6, r6, #2
 800769e:	d006      	beq.n	80076ae <__libc_init_array+0x4e>
 80076a0:	2400      	movs	r4, #0
 80076a2:	f855 3b04 	ldr.w	r3, [r5], #4
 80076a6:	3401      	adds	r4, #1
 80076a8:	4798      	blx	r3
 80076aa:	42a6      	cmp	r6, r4
 80076ac:	d1f9      	bne.n	80076a2 <__libc_init_array+0x42>
 80076ae:	bd70      	pop	{r4, r5, r6, pc}

080076b0 <MASS_DeviceDescriptor>:
 80076b0:	0112 0200 0000 4000 0483 5720 0200 0201     .......@.. W....
 80076c0:	0103 0000                                   ....

080076c4 <MASS_ConfigDescriptor>:
 80076c4:	0209 0020 0101 c000 0932 0004 0200 0608     .. .....2.......
 80076d4:	0450 0507 0281 0040 0700 0205 4002 0000     P.....@......@..

080076e4 <MASS_StringLangID>:
 80076e4:	0304 0409                                   ....

080076e8 <MASS_StringVendor>:
 80076e8:	0326 0053 0054 004d 0069 0063 0072 006f     &.S.T.M.i.c.r.o.
 80076f8:	0065 006c 0065 0063 0074 0072 006f 006e     e.l.e.c.t.r.o.n.
 8007708:	0069 0063 0073 0000                         i.c.s...

08007710 <MASS_StringProduct>:
 8007710:	0326 0053 0054 004d 0033 0032 0020 004d     &.S.T.M.3.2. .M.
 8007720:	0061 0073 0073 0020 0053 0074 006f 0072     a.s.s. .S.t.o.r.
 8007730:	0061 0067 0065 0000                         a.g.e...

08007738 <MASS_StringInterface>:
 8007738:	0310 0053 0054 0020 004d 0061 0073 0073     ..S.T. .M.a.s.s.

08007748 <HexChars>:
 8007748:	3130 3332 3534 3736 3938 4241 4443 4645     0123456789ABCDEF
 8007758:	0000 0000 6e28 6c69 0029 0000 7250 7365     ....(nil)...Pres
 8007768:	7573 6572 253a 0d66 000a 0000               sure:%f.....

08007774 <_init>:
 8007774:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
 8007776:	bf00      	nop
 8007778:	bcf8      	pop	{r3, r4, r5, r6, r7}
 800777a:	bc08      	pop	{r3}
 800777c:	469e      	mov	lr, r3
 800777e:	4770      	bx	lr

08007780 <__init_array_start>:
 8007780:	08000101 	.word	0x08000101

08007784 <_fini>:
 8007784:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
 8007786:	bf00      	nop
 8007788:	bcf8      	pop	{r3, r4, r5, r6, r7}
 800778a:	bc08      	pop	{r3}
 800778c:	469e      	mov	lr, r3
 800778e:	4770      	bx	lr

08007790 <__fini_array_start>:
 8007790:	080000ed 	.word	0x080000ed

08007794 <__cs3_regions>:
 8007794:	00000000 	.word	0x00000000
 8007798:	080077a8 	.word	0x080077a8
 800779c:	20000000 	.word	0x20000000
 80077a0:	00000170 	.word	0x00000170
 80077a4:	000003a8 	.word	0x000003a8
