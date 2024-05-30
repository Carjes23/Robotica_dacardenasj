/*
 * Single_Cell.c
 *
 *  Created on: Mar 8, 2024
 *      Author: namontoy
 *
 * Este archivo posee las funciones para crear celdas vacias (casilllas), de forma
 * que luego cada una de ellas pueda ser completada con la información que le pasamos
 * a nuestro equipo por medio de la comunicación RS-232 inalambrica.
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "Single_Cell.h"

/* Prototipos privados */
Cell_map_t init_empty_cell(Cell_map_t dummyCell);

/**
 * Esta funcion se encarga de inicializar las celdas que hacen parte
 * del grid del mapa que se desea estudiar.
 *
 * La idea es que cada celda debe ser capaz de reconocer a sus vecinos
 * por la posicion x,y que tienen en el grid_map.
 *
 * Como tal, al inicio, cada celda comienza completamente vacia y se va llenando a medida
 * que le enviamos la información del mapa a nuestro equipo.
*/
Cell_map_t init_empty_cell(Cell_map_t dummyCell){
    // aca se escribe toda la info de la celda...
    dummyCell.id = 255;
    dummyCell.pos_x = 255;
    dummyCell.pos_y = 255;
    dummyCell.type = 255;
    dummyCell.g = 255;
    dummyCell.h = 255;
    dummyCell.f = 255;
    dummyCell.parent = 255;
    dummyCell.state = NONE;
    for (uint8_t i = 0; i < 8; i++)
    {
        dummyCell.neighbors[i] = 255;
    }
    return dummyCell;
}

/*
 *	Esta función es utilizada para crear TODAS las celdas del mapa inicializadas
 *	como vacias pero en la posicion X, Y que se pasan como parametros, o sea, como
 *	tal esta función crea todos los elementos del mapa en sus posiciones correctas
 *	pero sin organizar sus caracteristicas mas específicas.
 *
 * */
Cell_map_t create_cell(uint8_t pos_x, uint8_t pos_y){
    // Se utiliza la funcion init_empty_cell(unknowCell2);
    
    Cell_map_t unknowCell = {0};
    unknowCell = init_empty_cell(unknowCell);
    unknowCell.pos_x = pos_x;
    unknowCell.pos_y = pos_y;
    return  unknowCell;
}
