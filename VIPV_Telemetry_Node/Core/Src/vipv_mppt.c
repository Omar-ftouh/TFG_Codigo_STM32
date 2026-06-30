/*
 * vipv_mppt.c
 *
 *  Created on: May 31, 2026
 *      Author: omarf
 */

#include "vipv_mppt.h"


// ----------------- EQUIVALENCIA A LIBRERÍA numpy DE PYTHON --------------------

// 1. Función para calcular la potencia directa (Interpolación Lineal)

float obtener_potencia_directa(float v_actual, const float *array_v, float *array_p, int tamano) {

    // Equivalencia a np.clip en Python: Evitar salir de los límites del array

	// Si el voltaje pedido es menor que el mínimo del array, se limita al mínimo
    if (v_actual <= array_v[0]) {
        return array_p[0];
    }
    // Si el voltaje pedido es mayor que el máximo, se limita al máximo
    if (v_actual >= array_v[tamano - 1]) {
        return array_p[tamano - 1];
    }


    // Equivalecia a np.interp en Python: Interpolación lineal

    // Búsqueda de entre qué dos puntos exactos del array nos encontramos
    int i = 0;
    while (i < tamano - 1 && v_actual > array_v[i + 1]) {
        i++;
    }

    	//Fórmula matemática de la ecuación de la recta entre 2 puntos:
        // P = P0 + (V_act - V0) * (P1 - P0) / (V1 - V0)
        float delta_v = array_v[i + 1] - array_v[i];
        if (delta_v == 0.0f) return array_p[i];

        float proporcion = (v_actual - array_v[i]) / delta_v;
        float p_interpolada = array_p[i] + proporcion * (array_p[i + 1] - array_p[i]);

        return p_interpolada;
}

/*
float obtener_potencia_panel(float v_actual, const float *v_datos, const float *i_datos, uint8_t tamano_array) {

    // Equivalencia a np.clip en Python: Evitar salir de los límites del array

	// Si el voltaje pedido es menor que el mínimo del array, se limita al mínimo
    if (v_actual <= v_datos[0]) {
        return v_actual * i_datos[0];
    }
    // Si el voltaje pedido es mayor que el máximo, se limita al máximo
    if (v_actual >= v_datos[tamano_array - 1]) {
        return v_actual * i_datos[tamano_array - 1];
    }


    // Equivalecia a np.interp en Python: Interpolación lineal

    // Búsqueda de entre qué dos puntos exactos del array nos encontramos
    for (uint8_t i = 0; i < tamano_array - 1; i++) {
        if (v_actual >= v_datos[i] && v_actual <= v_datos[i + 1]) {

        	//Fórmula matemática de la ecuación de la recta entre 2 puntos:
            float x0 = v_datos[i];
            float y0 = i_datos[i];
            float x1 = v_datos[i + 1];
            float y1 = i_datos[i + 1];

            float i_interpolada = y0 + ((v_actual - x0) * (y1 - y0)) / (x1 - x0);

            // Retornar potencia (P = V * I)
            return v_actual * i_interpolada;
        }
    }
    return 0.0f;
}
*/


// 2. Algoritmo MPPT (Perturbar y Observar)
float mppt_po(float v_actual, float p_actual, float v_anterior, float p_anterior, float paso_v) {

    float delta_p = p_actual - p_anterior;
    float delta_v = v_actual - v_anterior;
    float v_siguiente = v_actual;

    	// Si la potencia ha aumentado
        if (delta_p > 0) {
            if (delta_v > 0) {
                v_siguiente = v_actual + paso_v;  // Se mantiene el sentido (+)
            } else {
                v_siguiente = v_actual - paso_v;  // Se mantiene el sentido (-)
            }
        }
        // Si la potencia ha disminuido
        else {
            if (delta_v > 0) {
                v_siguiente = v_actual - paso_v;  // Se invierte el sentido (-)
            } else {
                v_siguiente = v_actual + paso_v;  // Se invierte el sentido (+)
            }
        }

    return v_siguiente;
}



float obtener_potencia_dinamica(float v_actual, float irr_actual) {

    // Acotación de la irradiancia para evitar fallos de cálculo en la interpolación
    if (irr_actual > IRR_MAX) irr_actual = IRR_MAX;
    if (irr_actual < IRR_MIN) irr_actual = IRR_MIN;


    // INTERPOLACIÓN LINEAL:

    // Potencia que daría el panel a pleno sol a este voltaje
    float p_v_sol = obtener_potencia_directa(v_actual, v_vector, p_sol, 50);
    // Potencia que daría a la sombra a este mismo voltaje
    float p_v_som = obtener_potencia_directa(v_actual, v_vector, p_sombra, 50);

    // Interpolación entre las dos curvas según la irradiancia actual
    float proporcion_luz = (irr_actual - IRR_MIN) / (IRR_MAX - IRR_MIN);
    float p_final_real = p_v_som + proporcion_luz * (p_v_sol - p_v_som);

    return p_final_real;
}


// Variables estáticas para que conserven su valor entre llamadas del bucle
static float irr_anterior = 0.0f;
static int contador_cambios = 0;
static int timer_congelacion = 0;


MPPT_Modo_t ejecutar_mppt_adaptativo(float *v_actual, float irr_actual, float v_anterior, float p_anterior, float paso_v, float p_actual) {
/*
    // Derivada de la irradiancia con respecto a la iteración anterior
    float delta_irr = irr_actual - irr_anterior;

    // Detección de flanco brusco (tanto de entrada a sombra como de salida a sol)
    if (fabsf(delta_irr) >= UMBRAL_BRUSCO) {
        contador_cambios++;
    }
    else {
        if (contador_cambios > 0 && timer_congelacion == 0) {
            // Cada segundo de estabilidad reduce el contador, pero no baja de cero
            static int ciclo_decae = 0;
            if(++ciclo_decae >= 3) { // Cada 3 segundos estables se decrementa el contador de cambios
                contador_cambios--;
                ciclo_decae = 0;
            }
        }
    }

    // Evaluar si se dispara la condición de congelación
    if (contador_cambios >= LIMITE_CAMBIOS && timer_congelacion == 0) {
        timer_congelacion = TIEMPO_CONGELACION;
        contador_cambios = 0; // Reinicio del contador para el siguiente ciclo
    }

    irr_anterior = irr_actual;


    // ---- MÁQUINA DE ESTADOS DEL MPPT ADAPTATIVO ----
    if (timer_congelacion > 0) {

        // 1) ESTADO: CONGELADO
    	*v_actual = VOLTAJE_PUNTO_FIJO;
        timer_congelacion--; // Decrementar el temporizador (1 segundo por vuelta del ciclo principal)
    }
    else {
        // 2) ESTADO: NORMAL
    }

    return MPPT_MODO_NORMAL;
    */

	// STATE 1: EL SISTEMA ESTÁ CONGELADO
	    if (timer_congelacion > 0) {
	        *v_actual = VOLTAJE_PUNTO_FIJO; // Forzamos los 19.5V seguros
	        timer_congelacion--;

	        // ¡EL BLINDAJE!: Forzamos el contador a cero continuamente para asegurar
	        // que al salir de la congelación empecemos con el historial limpio.
	        contador_cambios = 0;
	        irr_anterior = irr_actual;

	        return MPPT_MODO_CONGELADO;
	    }

	    // STATE 2: MODO NORMAL (Evaluación activa de transitorios lumínicos)
	    float delta_irr = irr_actual - irr_anterior;
	    irr_anterior = irr_actual;

	    if (fabsf(delta_irr) >= UMBRAL_BRUSCO) {
	        contador_cambios++;
	    } else {
	        // Amortiguación gradual si el cielo se estabiliza
	        if (contador_cambios > 0) {
	            static int ciclo_decae = 0;
	            if (++ciclo_decae >= 3) {
	                contador_cambios--;
	                ciclo_decae = 0;
	            }
	        }
	    }

	    // STATE 3: DETECCIÓN DE INESTABILIDAD (Disparo de congelación)
	    if (contador_cambios >= LIMITE_CAMBIOS) {
	        timer_congelacion = TIEMPO_CONGELACION;
	        contador_cambios = 0; // Limpiamos el historial de inmediato

	        *v_actual = VOLTAJE_PUNTO_FIJO;
	        timer_congelacion--;  // Consumimos el primer segundo de este ciclo

	        return MPPT_MODO_CONGELADO;
	    }

	    // Si everything está tranquilo, operamos en modo clásico
	    return MPPT_MODO_NORMAL;
}
