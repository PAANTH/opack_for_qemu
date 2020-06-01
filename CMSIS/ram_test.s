#
# Тест внутренней оперативной памяти микроконтроллера (той где выполняется программа)
# Вызов из Reset_Handler (startup_file.s)
#


#.include "mcu_build.h"

  .syntax unified
  .cpu cortex-m3
  .fpu softvfp
  .thumb

# Internal SRAM base address
.set STM_ISRAM, 0x20000000
# Internal SRAM size in byte (64 Kbytes)
.set STM_ISRAM_SIZE, 0x00010000
# Internal CCM base address
.set STM_ICCM, 0x10000000
# Internal SRAM size in byte (64 Kbytes) - 4 (Boot_val) //NOT FOR F107!!!
#.set STM_ICCM_SIZE, (0x00010000-4)
.set STM_ICCM_SIZE, (0x00008000-4)

.global ram_test

  .section  .text.ram_test
  .weak  ram_test
  .type  ram_test, %function
ram_test:

    ldr R1,=(STM_ISRAM + STM_ISRAM_SIZE)  /* последний адрес RAM */

/* Проверяем память на единицах */
    ldr R0,=STM_ISRAM            /* первый адрес RAM */
    /* Пропишем единицы */
    ldr R2,=0xFFFFFFFF
loop_ff:
    str R2,[R0]
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_ff

    /* Проверим */
    ldr R3,=0xFFFFFFFF
    ldr R0,=STM_ISRAM            /* первый адрес RAM */
loop_check_ff:
    ldr R2,[R0]                /* загрузим в регистр R2 из памяти по адресу в R0 */
    cmp  R2,R3                /* проверим */
    bne  err_ram_test            /* ошибка */
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_check_ff            /* продолжим проверку */

/* Проверим на шахматке */
    ldr R0,=STM_ISRAM            /* первый адрес RAM */
    /* Пропишем */
    ldr R2,=0xAAAAAAAA
loop_aa:
    str R2,[R0]
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_aa

    /* Проверим*/
    ldr R3,=0xAAAAAAAA
    ldr R0,=STM_ISRAM            /* первый адрес RAM */
loop_check_aa:
    ldr R2,[R0]                /* загрузим в регистр R2 из памяти по адресу в R0 */
    cmp  R2,R3                /* проверим */
    bne  err_ram_test            /* ошибка */
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_check_aa            /* продолжим проверку */

/* Проверим на счётчик (байтовый) */
    /* В R1 находится последний адрес - он не проверяется */
    ldr R0,=STM_ISRAM            /* первый адрес RAM */
    /* Пропишем адреса байт */
    ldr R2,=0
loop_addres:
    strb R2,[R0]
    add R0,R0,1              /* increase loop index */
    add R2,R2,1
    cmp R0,R1
    bne loop_addres

    /* Проверим */
    ldr R0,=STM_ISRAM            /* первый адрес RAM */
    ldr R3,=0
    ldr R4,=0x000000FF            /* mask */
loop_check_addres:
    ldrb R2,[R0]              /* загрузим в регистр R2 из памяти по адресу в R0 */
    cmp  R2,R3                /* проверим */
    bne  err_ram_test            /* ошибка */
    add R0,R0,1              /* increase loop index */
    add R3,R3,1
    and R3,R3,R4
    cmp R0,R1
    bne loop_check_addres          /* продолжим проверку */

/* Проверим на 0*/
    ldr R0,=STM_ISRAM            /* первый адрес RAM */
    /* Пропишем нули*/
    ldr R2,=0
loop_null:
    str R2,[R0]
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_null

    /* Проверим*/
    ldr R0,=STM_ISRAM            /* первый адрес RAM */
loop_check_null:
    ldr R2,[R0]                /* загрузим в регистр R2 из памяти по адресу в R0 */
    cmp  R2,0                /* проверим */
    bne  err_ram_test            /* ошибка */
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_check_null            /* продолжим проверку */

/*RAM_OK check ccm now*/
    bl  ram_test_ok

/*
  Ошибка в тесте SRAM. B Error_RAM_Test
*/
err_ram_test:
    ldr     R0, =Error_RAM_Test        /* go to main.c*/
    blx     R0

/*testCCM*/

test_ccm:
    ldr R1,=(STM_ICCM + STM_ICCM_SIZE)  /* последний адрес RAM */

/* Проверяем память на единицах */
    ldr R0,=STM_ICCM            /* первый адрес RAM */
    /* Пропишем единицы */
    ldr R2,=0xFFFFFFFF
loop_ff_2:
    str R2,[R0]
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_ff_2

    /* Проверим */
    ldr R3,=0xFFFFFFFF
    ldr R0,=STM_ICCM            /* первый адрес RAM */
loop_check_ff_2:
    ldr R2,[R0]                /* загрузим в регистр R2 из памяти по адресу в R0 */
    cmp  R2,R3                /* проверим */
    bne  err_ram_test_2            /* ошибка */
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_check_ff_2            /* продолжим проверку */

/* Проверим на шахматке */
    ldr R0,=STM_ICCM            /* первый адрес RAM */
    /* Пропишем */
    ldr R2,=0xAAAAAAAA
loop_aa_2:
    str R2,[R0]
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_aa_2

    /* Проверим */
    ldr R3,=0xAAAAAAAA
    ldr R0,=STM_ICCM            /* первый адрес RAM */
loop_check_aa_2:
    ldr R2,[R0]                /* загрузим в регистр R2 из памяти по адресу в R0 */
    cmp  R2,R3                /* проверим  */
    bne  err_ram_test_2            /* ошибка  */
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_check_aa_2            /* продолжим проверку */

/* Проверим на счётчик (байтовый) */
    /* В R1 находится последний адрес - он не проверяется */
    ldr R0,=STM_ICCM            /* первый адрес RAM */
    /* Пропишем адреса байт */
    ldr R2,=0
loop_addr_2:
    strb R2,[R0]
    add R0,R0,1              /* increase loop index */
    add R2,R2,1
    cmp R0,R1
    bne loop_addr_2

    /* Проверим */
    ldr R0,=STM_ICCM            /* первый адрес RAM */
    ldr R3,=0
    ldr R4,=0x000000FF            /* mask */
loop_check_addr_2:
    ldrb R2,[R0]              /* загрузим в регистр R2 из памяти по адресу в R0 */
    cmp  R2,R3                /* проверим */
    bne  err_ram_test_2            /* ошибка */
    add R0,R0,1              /* increase loop index */
    add R3,R3,1
    AND R3,R3,R4
    cmp R0,R1
    bne loop_check_addr_2          /* продолжим проверку */

/* Проверим на 0 */
    ldr R0,=STM_ICCM            /* первый адрес RAM */
    /* Пропишем нули */
    ldr R2,=0
loop_null_2:
    str R2,[R0]
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_null_2

    /* Проверим */
    ldr R0,=STM_ICCM            /* первый адрес RAM */
loop_check_null_2:
    ldr R2,[R0]                /* загрузим в регистр R2 из памяти по адресу в R0 */
    cmp  R2,0                /* проверим */
    bne  err_ram_test_2            /* ошибка */
    add R0,R0,4              /* increase loop index */
    cmp R0,R1
    bne loop_check_null_2            /* продолжим проверку */

    /*RAM test all ok go back*/
    bl  ram_test_ok

/*
  Ошибка в тесте CCM. B Error_RAM_Test
*/
err_ram_test_2:
#err_ram_test:
    ldr     R0, =Error_RAM_Test        /* go to main.c*/
    blx     R0
