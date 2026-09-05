#pragma once
#include <stdio.h>
#define LOG_LEVEL_INF 0
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) do { printf("[inf] " __VA_ARGS__); printf("\n"); } while (0)
#define LOG_WRN(...) do { printf("[wrn] " __VA_ARGS__); printf("\n"); } while (0)
#define LOG_ERR(...) do { printf("[err] " __VA_ARGS__); printf("\n"); } while (0)
