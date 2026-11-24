#pragma once
#include <cassert>
#include <cstdint>
using esp_err_t=int;
constexpr esp_err_t ESP_OK=0;
inline const char *esp_err_to_name(int){return "FAKE";}
struct Cmp {int value=1000;};
using mcpwm_cmpr_handle_t=Cmp*;
using mcpwm_gen_handle_t=int*;
using mcpwm_timer_handle_t=int*;
using mcpwm_oper_handle_t=int*;
constexpr int MCPWM_TIMER_CLK_SRC_DEFAULT=0,MCPWM_TIMER_COUNT_MODE_UP=0,MCPWM_TIMER_START_NO_STOP=0;
struct mcpwm_timer_config_t {int group_id,clk_src,resolution_hz,count_mode,period_ticks;};
struct mcpwm_operator_config_t {int group_id;};
struct mcpwm_comparator_config_t {struct {bool update_cmp_on_tez;}flags;};
struct mcpwm_generator_config_t {int gen_gpio_num;};
#define MCPWM_GEN_TIMER_EVENT_ACTION(...) 0
#define MCPWM_GEN_COMPARE_EVENT_ACTION(...) 0
inline int mcpwm_new_timer(mcpwm_timer_config_t*,int**p){*p=new int;return 0;}
inline int mcpwm_new_operator(mcpwm_operator_config_t*,int**p){*p=new int;return 0;}
inline int mcpwm_operator_connect_timer(int*,int*){return 0;}
inline int mcpwm_new_comparator(int*,mcpwm_comparator_config_t*,Cmp**p){*p=new Cmp;return 0;}
inline int mcpwm_comparator_set_compare_value(Cmp*p,int v){assert(v>=1000&&v<=1600);p->value=v;return 0;}
inline int mcpwm_new_generator(int*,mcpwm_generator_config_t*,int**p){*p=new int;return 0;}
inline int mcpwm_generator_set_action_on_timer_event(int*,int){return 0;}
inline int mcpwm_generator_set_action_on_compare_event(int*,int){return 0;}
inline int mcpwm_timer_enable(int*){return 0;}
inline int mcpwm_timer_start_stop(int*,int){return 0;}
inline int mcpwm_generator_set_force_level(int*,int,bool){return 0;}
