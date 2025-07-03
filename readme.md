## 以stm32f103c8t6为例，了解代码是如何被编译出来的

![image-20250703151007791](https://newbie-typora.oss-cn-shenzhen.aliyuncs.com/TyporaJPG/image-20250703151007791.png)

## 其中 start.s（汇编文件）的预处理产物

有两种情况要区分：

（A）**GNU 风格 .S 文件（注意后缀大写）**

- **`.S`（大写 S）**：是**需要预处理的汇编文件**
- 可以用 C 语言的预处理功能，比如 `#define`、`#include`，这样编译器会先进行预处理（比如头文件展开、宏替换），输出为**纯汇编代码文件**（例如 `start.s` 经过预处理后生成 `start.s`/`start.i`/`start.spp` 视具体命令而定）

（B）**.s（小写 s）文件**

- 纯汇编，不做预处理，直接交给汇编器











这里以点灯为例,使用cubemx生成一个hal库最小点灯 keil工程，再将工程裁剪，使其不依赖hal库，得到最精简的代码，让led闪烁

- cubemx配置HSE外部时钟，使能LED时钟及其配置
- 验证led是否正常闪烁
- 精简代码不依赖hal库,全部使用寄存器配置并验证

## keil工程

精简后最终得到如下文件   见project:   2.minimum_program\mini-stm32-f103c8t6

```c
|-- core_cm3.h
|-- main.c
|-- startup_stm32f103xb.s
|-- stm32f103xb.h
```

- 其中core_cm3.h  与 stm32f103xb.h 定义了寄存器的结构体以及配置宏,  全写main.c里太多了，这里分开
- 实际上只需要一个.s启动文件与 mian函数所在的文件.c  此外还有一个隐藏的文件，链接脚本，就可以让代码跑起来了(.s .c link_file)
- 链接脚本查看如下
- keil的链接脚本也叫做分散加载文件，是以sct结尾的文件，语法参考链接 https://developer.arm.com/documentation/dui0474/m/scatter-file-syntax/syntax-of-a-scatter-file?lang=en

![image-20250703153156078](https://newbie-typora.oss-cn-shenzhen.aliyuncs.com/TyporaJPG/image-20250703153156078.png)

![image-20250703153408593](https://newbie-typora.oss-cn-shenzhen.aliyuncs.com/TyporaJPG/image-20250703153408593.png)

![image-20250703154337632](https://newbie-typora.oss-cn-shenzhen.aliyuncs.com/TyporaJPG/image-20250703154337632.png)

## arm-none-eabi-gcc编译工具链手工编译

准备文件

```
|-- core_cm3.h
|-- main.c
|-- startup_stm32f103xb.s
|-- stm32f103xb.h
|-- STM32F103XX_FLASH.ld
```

- 对于arm-none-eabi-gcc 编译工具链，启动文件.s的语法不一样，以及链接脚本的语法也不一样为 后缀为 ld 一般称为链接脚本
- 直接使用cubemx生成的cmake 工程   startup_stm32f103xb.s 和 STM32F103XX_FLASH.ld 拿过来直接用

准备好上述一共五个文件，安装好arm-none-eabi-gcc编译工具链



### 1. **编译 C 文件为目标文件（.o）**   

此处直接编译成目标文件，（实际上可以体验 预处理，编译，汇编的过程）

```
arm-none-eabi-gcc -mcpu=cortex-m3 -fdata-sections -ffunction-sections -O0 -g3 -std=gnu11 -c main.c -o main.c.o
```

- **作用**：把 `main.c` 编译成目标文件 `main.c.o`，用于后续链接。
- **常用参数：**
  - `-mcpu=cortex-m3`：指定目标 CPU 架构为 Cortex-M3。
  - `-fdata-sections -ffunction-sections`：让每个数据/函数单独成段，便于后续去除未用部分。
  - `-O0`：关闭优化，便于调试。
  - `-g3`：生成完整的调试信息。
  - `-std=gnu11`：C 标准选择 GNU C11。
  - `-c`：只编译，不链接。
  - `-o main.c.o`：指定输出目标文件。

------

### 2. **汇编启动文件为目标文件（.o）**

```
arm-none-eabi-gcc -mcpu=cortex-m3 -x assembler-with-cpp -MMD -MP -g -c startup_stm32f103xb.s -o startup_stm32f103xb.s.o
```

- **作用**：把启动文件 `startup_stm32f103xb.s` 汇编成目标文件 `startup_stm32f103xb.s.o`。
- **常用参数：**
  - `-x assembler-with-cpp`：启用汇编文件的 C 预处理能力。
  - `-MMD -MP`：生成依赖文件，方便增量编译（Makefile 用）。
  - 其余参数同上。

------

### 3. **链接所有目标文件为可执行文件（.elf）**

```
arm-none-eabi-gcc -mcpu=cortex-m3 \
  -T STM32F103XX_FLASH.ld --specs=nano.specs \
  -Wl,-Map=103c8-cmake.map -Wl,--gc-sections \
  -Wl,--start-group main.c.o startup_stm32f103xb.s.o -lc -lm -Wl,--end-group \
  -Wl,--print-memory-usage -o 103c8-cmake.elf
```

- **作用**：把所有 `.o` 文件链接成 `103c8-cmake.elf`，带调试信息的可执行文件。
- **关键参数：**
  - `-T STM32F103XX_FLASH.ld`：指定链接脚本，决定代码/数据段的实际内存分布。
  - `--specs=nano.specs`：使用精简的 newlib-nano C 库。
  - `-Wl,xxx`：传递参数给链接器（ld）。
    - `-Map=103c8-cmake.map`：生成映射文件。
    - `--gc-sections`：自动去除未引用的段（配合 `-fdata-sections`）。
    - `--start-group ... --end-group`：解决多库依赖问题。
    - `-lc -lm`：链接 C 标准库和数学库。
    - `--print-memory-usage`：链接后打印 RAM/FLASH 使用情况。
  - `-o 103c8-cmake.elf`：指定输出的 ELF 文件。

------

### 4. **格式转换为 HEX/BIN（烧录文件）**

```
arm-none-eabi-objcopy -O ihex 103c8-cmake.elf 103c8-cmake.hex
arm-none-eabi-objcopy -O binary 103c8-cmake.elf 103c8-cmake.bin
```

- **作用**：把 ELF 文件转换为更适合烧录/传输的镜像格式。
  - `-O ihex`：生成 Intel HEX 文件（文本，广泛兼容于烧录工具）。
  - `-O binary`：生成裸二进制 BIN 文件（直接烧录到 Flash）。
- **用途**：
  - `.hex` 和 `.bin` 都是实际烧录 MCU 常用的格式，依据烧录工具和厂商不同选用。

------

### **总结/流程一览**

1. **编译**：.c → .o
2. **汇编**：.s → .o
3. **链接**：所有 .o → .elf（开发/调试用，带符号）
4. **格式转换**：.elf → .hex/.bin（烧录/量产用）



### 当然也可以一条命令直接得到全部

```
arm-none-eabi-gcc -mcpu=cortex-m3 -fdata-sections -ffunction-sections -O0 -g3 -std=gnu11 \
  -T STM32F103XX_FLASH.ld --specs=nano.specs \
  -Wl,-Map=103c8-cmake.map -Wl,--gc-sections \
  -Wl,--start-group main.c startup_stm32f103xb.s -lc -lm -Wl,--end-group \
  -Wl,--print-memory-usage -o 103c8-cmake.elf \
  && arm-none-eabi-objcopy -O ihex 103c8-cmake.elf 103c8-cmake.hex \
  && arm-none-eabi-objcopy -O binary 103c8-cmake.elf 103c8-cmake.bin
```



![image-20250703160303958](https://newbie-typora.oss-cn-shenzhen.aliyuncs.com/TyporaJPG/image-20250703160303958.png)



## 此外还更改了cmake工程

- 其中mini-stm32-f103c8t6-cmake 将链接脚本放在src目录，可体验手工编译

```bash
|-- 1.original_program         
|   |-- stm32-f103c8t6          //cubemx生成原始keil工程
|   `-- stm32-f103c8t6-cmake    //cubemx生成原始cmaek工程
|-- 2.minimum_program
|   |-- mini-stm32-f103c8t6       //精简后的keil工程
|   `-- mini-stm32-f103c8t6-cmake //精简后的cmake工程

```



个人感想：

大学时直接学stm32,一上来就一大堆文件，无从下手，对这一部分一知半解，现在有空梳理下这部分知识

- 这种开发方式应该是很多年前的开发方式吧，直接配置寄存器，现在各个厂商都封装好了库直接用，不用看底层的寄存器，读库的说明也可以开发了，可移植性提高了，门槛也降低了，
- 当然一些低端小内存芯片还是用原始的寄存器开发，见过台湾九齐OTP芯片,ram就几个字节，没听错，几个字节
- 手工编译，makefile,  cmake，工程管理也在变得更加自动化