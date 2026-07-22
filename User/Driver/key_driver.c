#include "key_driver.h"

/* 定义默认设置 */
static const ebtn_btn_param_t defaul_ebtn_param = EBTN_PARAMS_INIT(
    20,     // time_debounce: 按下稳定 20ms
    20,     // time_debounce_release: 释放稳定 20ms
    50,     // time_click_pressed_min: 最短单击按下 50ms，配置为0，代表不检查最小值
    500,    // time_click_pressed_max: 最长单击按下 500ms (超过则不算单击)，配置为0xFFFF，代表不检查最大值，用于区分长按和按键事件。
    200,    // time_click_multi_max: 多次单击最大间隔 300ms (两次点击间隔超过则重新计数)
    200,    // time_keepalive_period: 长按事件周期 500ms (按下超过 500ms 后，每 500ms 触发一次)
    5       // max_consecutive: 最多支持 5 连击
);



/* 2. 定义静态按键列表 */
// 宏: EBTN_BUTTON_INIT(按键ID, 参数指针)
static ebtn_btn_t static_buttons[] = {
        EBTN_BUTTON_INIT(USER_BUTTON_1, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_BUTTON_2, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_BUTTON_3, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_BUTTON_4, &defaul_ebtn_param),
  
        EBTN_BUTTON_INIT(USER_PS2_UP, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_PS2_DOWN, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_PS2_LEFT, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_PS2_RIGHT, &defaul_ebtn_param),
};

// /* 3. 定义静态组合按键列表 (可选) */
// // 宏: EBTN_BUTTON_COMBO_INIT(按键ID, 参数指针)
// ebtn_btn_combo_t static_combos[] = {
//     // 假设 KEY1+KEY2 组合键
//     EBTN_BUTTON_COMBO_INIT(USER_BUTTON_COMBO_0, &defaul_ebtn_param), // 组合键, ID=USER_BUTTON_COMBO_0 (必须与普通按键ID不同)
  
//     EBTN_BUTTON_COMBO_INIT(USER_BUTTON_COMBO_1, &defaul_ebtn_param), 
  
//     EBTN_BUTTON_COMBO_INIT(USER_BUTTON_COMBO_2, &defaul_ebtn_param), 
// };

/* 1. 实现获取按键状态的回调函数 */
// 函数原型: uint8_t (*ebtn_get_state_fn)(struct ebtn_btn *btn);
uint8_t my_get_key_state(struct ebtn_btn *btn) {
    // 根据传入的按钮实例中的 key_id 判断是哪个物理按键
    switch (btn->key_id) {
        case 1: // 请求读取 KEY1 的状态
            // 假设按下为高电平 (返回 1 代表按下)
            return (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);
        case 2: 
          return (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET);
        case 3: 
          return (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET);
        case 4: 
          return (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET);
        case USER_PS2_UP: 
          return (PS2_DataKey() == 5) || (Get_Blue_Key() == 0x11);
        case USER_PS2_DOWN: 
          return (PS2_DataKey() == 7) || (Get_Blue_Key() == 0x22);
        case USER_PS2_LEFT: 
          return (PS2_DataKey() == 8) || (Get_Blue_Key() == 0x33);
        case USER_PS2_RIGHT: 
          return (PS2_DataKey() == 6) || (Get_Blue_Key() == 0x44);      
        // ... 可以添加更多按键的读取逻辑 ...
        default:
            // 对于库内部处理组合键等情况，或者未知的 key_id，安全起见返回 0 (未按下)
            return 0;
    }
    // 注意：返回值 1 表示 "活动/按下"，0 表示 "非活动/释放"
}

int Ebtn_Init(void)
{
  // 初始化 ebtn 库
    int init_ok = ebtn_init(
        static_buttons,                 // 静态按键数组的指针
        EBTN_ARRAY_SIZE(static_buttons), // 静态按键数量 (用宏计算)
        NULL,// static_combos,                  // 静态组合按键数组的指针 (如果没有，传 NULL, 0)
        0,// EBTN_ARRAY_SIZE(static_combos), // 静态组合按键数量 (如果没有，传 0)
        my_get_key_state,               // 你的状态获取回调函数
        my_handle_key_event             // 你的事件处理回调函数
    );

    if (!init_ok) {
        // 初始化失败，可能是参数错误
        return -1; // 或者进行其他错误处理
    }
    
    // 启用组合键优先处理模式，防止组合键和单键冲突
    ebtn_set_config(EBTN_CFG_COMBO_PRIORITY);

    // // --- 配置组合键 (如果使用了组合键) ---
    // // 1. 找到参与组合的普通按键的内部索引 (Index)
    // //    注意：这个内部索引不一定等于你设置的 key_id！
    // int key1_index = ebtn_get_btn_index_by_key_id(1); // 获取 KEY1 (ID=1) 的内部索引
    // int key2_index = ebtn_get_btn_index_by_key_id(2); // 获取 KEY2 (ID=2) 的内部索引

    // // 2. 将这些索引对应的按键添加到组合键定义中
    // //    确保索引有效 (>= 0)
    // if (key1_index >= 0 && key2_index >= 0) {
    //     // 假设 static_combos[0] 是我们定义的 ID=101 的组合键
    //     ebtn_combo_btn_add_btn_by_idx(&static_combos[0], key1_index); // 将 KEY1 添加到组合键
    //     ebtn_combo_btn_add_btn_by_idx(&static_combos[0], key2_index); // 将 KEY2 添加到组合键
    // } 
    // else {
    //     // 可能需要检查 key_id 是否正确，或者 ebtn_init 是否成功
    // }
    
    return 0;
}

unsigned char blue_key = 0;

void Blue_Key_Parse(unsigned char* buffer)
{
  if(buffer[0] == 0x55 && buffer[2] == 0x55)
  {
    blue_key = buffer[1];
  }
}

void Reset_Blue_Key(void)
{
  blue_key = 0;
}

unsigned char Get_Blue_Key(void)
{
  return blue_key;
}
