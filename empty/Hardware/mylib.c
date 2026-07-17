/**
 * 整数转字符串
 * num  : 输入整数 (int32_t)
 * str  : 输出缓冲区 (至少12字节)
 * base : 进制 (10或16)
 */
 #include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

void my_itoa(int32_t num, char *str, uint8_t base) {
    int32_t i = 0;
    int32_t isNegative = 0;

    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return;
    }
    if (num < 0 && base == 10) {
        isNegative = 1;
        num = -num;
    }
    while (num != 0) {
        int32_t rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10 + 'A') : rem + '0';
        num = num / base;
    }
    if (isNegative)
        str[i++] = '-';
    str[i] = '\0';

    // 反转字符串
    int32_t start = 0;
    int32_t end = i - 1;
    while (start < end) {
        char tmp = str[start];
        str[start] = str[end];
        str[end] = tmp;
        start++;
        end--;
    }
}

/*
 * 拼接字符串到 buf，返回当前写入位置
 * dst  : 目标缓冲区
 * src  : 源字符串
 * 返回 : 指向目标缓冲区当前末尾 '\0' 的指针
 */
char* my_strcat(char *dst, const char *src) {
    while (*dst) dst++;
    while (*src) *dst++ = *src++;
    *dst = '\0';
    return dst;
}

void IntToStr(int num, char *str) {
    int i = 0;
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return;
    }
    if (num < 0) {
        str[i++] = '-';
        num = -num;
    }
    int start = i;
    while (num > 0) {
        str[i++] = (num % 10) + '0';
        num /= 10;
    }
    str[i] = '\0';
    // 反转数字部分
    for (int a = start, b = i - 1; a < b; a++, b--) {
        char tmp = str[a];
        str[a] = str[b];
        str[b] = tmp;
    }
}