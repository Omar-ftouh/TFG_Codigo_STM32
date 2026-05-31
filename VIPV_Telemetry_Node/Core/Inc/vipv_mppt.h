/*
 * vipv_mppt.h
 *
 *  Created on: May 31, 2026
 *      Author: omarf
 */

#ifndef INC_VIPV_MPPT_H_
#define INC_VIPV_MPPT_H_

#include <stdint.h>

float obtener_potencia_panel(float v_actual, const float *v_datos, const float *i_datos, uint8_t tamano_array);
float mppt_po(float v_actual, float p_actual, float v_anterior, float p_anterior, float paso_v);

#endif /* INC_VIPV_MPPT_H_ */
