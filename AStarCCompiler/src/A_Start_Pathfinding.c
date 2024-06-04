/*
 ============================================================================
 Name        : A_star_pathfinding.c
 Author      : namontoy
 Version     :
 Copyright   : Programming to learn A*
 Dkescription : Learning how to write my first A*...
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "Single_Cell.h"

#define MAP_GRID_ROWS 8
#define MAP_GRID_COLS 10
#define MAP_CELLS_COUNT (MAP_GRID_ROWS * MAP_GRID_COLS)
#define ROW_MAP_DATA_LEN 20
#define MAX_NEIGHBOURS 8
#define GRID_SIZE 10

/* Elementos del sistema */
uint8_t grid_rows = MAP_GRID_ROWS;
uint8_t grid_cols = MAP_GRID_COLS;

Cell_map_t grid_map_cells[MAP_CELLS_COUNT] = {0};
Cell_map_t *ptr_current_cell;
Cell_map_t *ptr_goal_cell;
Cell_map_t *ptr_start_cell;

Cell_map_t *open_list[MAP_CELLS_COUNT];
uint8_t open_list_index = 0;

Cell_map_t *closed_list[MAP_CELLS_COUNT];
uint8_t closed_list_index = 0;

char *map_string[MAP_GRID_ROWS];

/* Prototipos de las funciones del main */
void init_empty_grid_map(uint8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray);
void print_cells_info(Cell_map_t *cellArray);
void print_single_cell_info(Cell_map_t *singleCell);
void print_map(int8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray);
void populate_grid(char *row_data, uint8_t grid_row, Cell_map_t *grid_map);
Cell_map_t *get_cell_start(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows);
Cell_map_t *get_cell_goal(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows);
void addTo_open_list(Cell_map_t *working_cell);
uint8_t identify_cell_neighbours(Cell_map_t *grid_map, Cell_map_t *cell_to_check);
uint16_t get_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
void update_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
uint16_t get_G_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
void update_G_cost(Cell_map_t *parent_cell, Cell_map_t *working_cell);
uint16_t get_F_cost(Cell_map_t *working_cell);
void update_F_cost(Cell_map_t *working_cell);
void order_open_list_last_elemnt(uint8_t index_last);
void init_empty_openlist(Cell_map_t *empty_cell);
uint8_t get_count_item_open_list(void);
void removeFrom_open_list(Cell_map_t *working_cell);
Cell_map_t *get_next_item(void);
void print_path(Cell_map_t *working_cell);
void A_star_algorithm(void);
extern Cell_map_t create_cell(uint8_t pos_x, uint8_t pos_y);

/**
 * Función que ejecuta paso a paso el algoritmo A*
 * */
int main(void)
{

	/* prints !!!Hello World!!! */
	printf("!!!Hello World!!!\n");

	printf("A* pathfinding\n");

	/* 1. Crea todas las celdas vacias */
	init_empty_grid_map(grid_cols, grid_rows, grid_map_cells);

	/* Inicializo todo el arreglo de punteros del open_list apuntando a la empty_cell */
	// init_empty_openlist(&empty_cell);

	/* 2. Llena el mapa con la descripcion para el ejercicio
	 * En el caso del MCU, el string se debe recibir por el puerto serial,
	 * al igual que la indicación de a qué fila del mapa corresponde
	 * */
	populate_grid("G . . . . # . . . . ", 0, grid_map_cells);
	populate_grid("# # # # # . . # # . ", 1, grid_map_cells);
	populate_grid("S . # . # # # # . . ", 2, grid_map_cells);
	populate_grid(". . # . . . . # . # ", 3, grid_map_cells);
	populate_grid("# . # . . . . # . . ", 4, grid_map_cells);
	populate_grid(". . # . . . . # # . ", 5, grid_map_cells);
	populate_grid(". . # . . . . # # . ", 6, grid_map_cells);
	populate_grid(". . . . . . . . . . ", 7, grid_map_cells);

	// Imprime la informacion mas simple de todas las celdas del grid
	print_cells_info(grid_map_cells);

	/* 3. Imprime en pantalla el mapa que se envió a por los comandos del USART,
	 * para verificar que en efecto el sistema tiene el mapa correcto, o que el mapa
	 * fue correctamente recibido
	 * */
	print_map(grid_cols, grid_rows, grid_map_cells);

	/* 4. Ejecución del algoritmo A*
	 * Al llamar esta funcion, que basicamente ejecuta el pseudocodigo, se debe
	 * obtener al final la solución para la ruta.
	 * */
	A_star_algorithm();

	printf("END\n");

	return 0;
}

/**
 * Esta funcion se encarga de devolver la distancia "directa" entre la celda X en la que se
 * está trabajando y la celda definida como "Goal". En este primer caso será una heuristica
 * del tipo Pitagoras h = sqrt( (X1 - X2)^2 + (Y1 - Y2)^2 ).
 * Con este valor debera ser suficiente.
 *
 * NOTA: Observar que para este calculo es preferible tener la unidad FPU activa en el MCU
 * además de ser mas conveniente hacer los calculos con las funciones de CMSIS que con las
 * de C standar ya que son mucho mas eficientes.
 * */
uint16_t get_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell)
{
	if (goal_cell == NULL || working_cell == NULL)
	{
		// Handle null pointer error (e.g., return a special value or log an error)
		return 0;
	}

	int16_t dx = goal_cell->pos_x - working_cell->pos_x;
	int16_t dy = goal_cell->pos_y - working_cell->pos_y;
	uint16_t aux_result = (uint16_t)sqrt(dx * dx + dy * dy);

	return aux_result;
}

/*
 * Actualiza el valor del H_cost de la celda que se pasa como parametro, con respecto
 * a la celda "Goal" que se pasa tambien como parametro
 * */
void update_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell)
{
	working_cell->h = get_H_cost(goal_cell, working_cell);
}

/**
 * Esta funcion se encarga de devolver la distancia "directa" entre la celda X y su siguiente
 * vecino, lo cual es el costo de viaje entre ambos. En este primer caso será una heuristica
 * del tipo Pitagoras G_cost_cell = sqrt( (X1 - X2)^2 + (Y1 - Y2)^2 ) + G_cost_vecino.
 *
 * Con este valor debera ser suficiente.
 *
 * NOTA: Observar que para este calculo es preferible tener la unidad FPU activa en el MCU
 * además de ser mas conveniente hacer los calculos con las funciones de CMSIS que con las
 * de C standar ya que son mucho mas eficientes.
 * */
uint16_t get_G_cost(Cell_map_t *neighbour_cell, Cell_map_t *working_cell)
{
	if (neighbour_cell == NULL || working_cell == NULL)
	{
		// Handle null pointer error (e.g., return a special value or log an error)
		return 0;
	}

	int16_t dx = neighbour_cell->pos_x - working_cell->pos_x;
	int16_t dy = neighbour_cell->pos_y - working_cell->pos_y;
	uint16_t distance_cost = (uint16_t)sqrt(dx * dx + dy * dy);
	uint16_t aux_result = distance_cost + working_cell->g;

	return aux_result;
}

/*
 * Actualiza el valor del G_cost de la celda que se pasa como parametro, con respecto
 * a la celda vecina que se pasa tambien como parametro
 *
 * Para el caso especial de la celda "star" el G_cost siempre es 0 (ya que desde allí se inicia).
 * Esta caracterisitica debe quedar incluida en esta funcion.
 * */
void update_G_cost(Cell_map_t *parent_cell, Cell_map_t *working_cell)
{

	working_cell->g = get_G_cost(parent_cell, working_cell);
}

/**
 * Esta funcion se encarga de retornar el valor de la funcion de costo completa de la celda que se
 * está analizando.
 *
 * F = G_cost + H_cost
 *
 * */
uint16_t get_F_cost(Cell_map_t *working_cell)
{
	uint16_t aux = working_cell->g + working_cell->h;
	return aux;
}

/*
 * Actualiza el valor F_cost de la celda que se pasa como parámetro
 * */
void update_F_cost(Cell_map_t *working_cell)
{

	working_cell->f = get_F_cost(working_cell);
}

/**
 * Esta funcion busca cual es la celda que esta designada como el inicio (start)
 * y se apunta a ella con el putero "current_cell" para comenzar a hacer el analisis,
 * además se le organizan los parametros G, H y F adecuadamente...
 *
 * Este es de los primeros paso del pseudocódigo
 *
 * Se imprime un error si la celda no es encontrada en el arreglo.
 *
 * */
Cell_map_t *get_cell_start(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows)
{

	if (ptr_start_cell != NULL)
	{
		ptr_current_cell = ptr_start_cell;
		ptr_start_cell->g = 0;
		ptr_start_cell->h = get_H_cost(ptr_goal_cell, ptr_start_cell);
		ptr_start_cell->f = get_F_cost(ptr_start_cell);
		return ptr_start_cell;
	}
	else
	{
		return NULL;
	}
}

/**
 * Esta funcion busca cual es la celda que esta designada como el "objetivo" (Goal),
 * además se le organizan los parametros G, H y F adecuadamente...
 * */
Cell_map_t *get_cell_goal(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows)
{
	if (ptr_goal_cell != NULL)
	{
		ptr_goal_cell->h = 0;
		ptr_goal_cell->f = 0;
		ptr_goal_cell->g = 0;
		return ptr_goal_cell;
	}
	else
	{
		return NULL;
	}
}

/**
 * Esta funcion debe organizar los elementos en la lista en orden, y el criterio es
 * primero F_cost y luego H_cost, si ambos son igual, se deja quien estaba primero.
 *
 * Este sistema implica que se debe poder mover hacia arriba y hacia abajo los elementos
 * dependiendo de la posicion que se le deba dar.
 *
 * Esta funcion utiliza un index, el cual solo puede ser modificado desde esta funcion
 * ya que este index se encarga de "mover" arriba y abajo el indicador de cuantos
 * elementos se encuentran abiertos.
 *
 * Además, tambien se tiene un puntero auxiliar, para poder comparar entre elementos y
 * saber quien debe ir en que posicion.
 *
 * Recordad que el elemento "open_list" es un arreglo de punteros de tipo Cell_map_t,
 * o sea esta lista NO almacena de nuevo las celdas, solo almacena la referencia a ellas
 * en el orden adecuado... punteros, queridos punteros...
 * */
void addTo_open_list(Cell_map_t *working_cell)
{
	working_cell->state = OPEN;
	open_list[open_list_index] = working_cell;
	order_open_list_last_elemnt(open_list_index);
	open_list_index++;
}

/*
 * Entrega el puntero al elemento mas arriba de la lista open_list
 * */
Cell_map_t *get_next_item(void)
{

	return open_list[0];
}

/**
 * Esta funcion identifica cuantos elementos activos hay en la lista open_list */
uint8_t get_count_item_open_list(void)
{

	uint8_t count = 0;
	for (int i = 0; i < MAP_CELLS_COUNT; i++)
	{
		if (open_list[i] != NULL)
		{
			count++;
		}
	}
	return count;
}
/*
 * Remueve el elemento indicado por el puntero working_cell, que ademas deberia ser el elemento
 * mas arriba en la lista de open_list
 * */
void removeFrom_open_list(Cell_map_t *working_cell)
{
	uint8_t aux = 0;
	uint8_t aux2 = 0;
	open_list_index--;
	if (open_list[0] == working_cell)
	{
		aux = 1;
	}
	else
	{
		for (int i = 0; i < MAP_CELLS_COUNT; i++)
		{
			if (open_list[i] == working_cell)
			{
				aux2 = 1;
			}
			if (aux2)
			{
				if (open_list[i + 1] != NULL)
				{
					open_list[i] = open_list[i + 1];
				}
				else
				{
					open_list[i] = NULL;
					break;
				}
			}
		}
	}
	if (aux)
	{
		for (int i = 0; i < MAP_CELLS_COUNT; i++)
		{
			if (open_list[i + 1] != NULL)
			{
				open_list[i] = open_list[i + 1];
			}
			else
			{
				open_list[i] = NULL;
				break;
			}
		}
	}
}

/*
 * Agrega el elemento pasado como parametro (puntero) en la lista de elementos cerrados
 * */
void addTo_closed_list(Cell_map_t *working_cell)
{
	working_cell->state = CLOSE;
	closed_list[closed_list_index] = working_cell;
	closed_list_index++;
}

/**
 * Carga la lista Open con el valor conocido de una celda vacia, y que además está fuera de las
 * fronteras del mapa. Esto lo hago con el objetivo de saber hasta donde debo recorrer el
 * arreglo open_list, de forma que pueda saber donde ya no hay nuevos elementos.

 * Es como la idea del caracter "NULL" en los strings...
 * */
void init_empty_openlist(Cell_map_t *empty_cell)
{

	open_list[0] = empty_cell;
}

/**
 * Funcion encargada de ordenear el arreglo OpenList con respecto a los valores F y H
 * de cada elemento al que se apunta en la lista.
 *
 * Esta función es "llamada" desde la función "addTo_open_list(...)"
 * */
void order_open_list_last_elemnt(uint8_t index_last)
{

	for (int i = 0; i < index_last; i++)
	{
		if (open_list[i]->f > open_list[index_last]->f)
		{
			Cell_map_t *aux = open_list[i];
			open_list[i] = open_list[index_last];
			open_list[index_last] = aux;
			break;
		}
		else if (open_list[i]->f == open_list[index_last]->f)
		{
			if (open_list[i]->h > open_list[index_last]->h)
			{
				Cell_map_t *aux = open_list[i];
				open_list[i] = open_list[index_last];
				open_list[index_last] = aux;
				break;
			}
		}
	}
}

/**
 * Un paso fundamental en el Algoritmo es identificar a los vecinos alrededor de la celda
 * que se encuentra activa en ese momento.
 * En este caso, deseo utilizar la posicion x,y como guia para encontrar los vecinos alrededor
 * de la celda[IDx] selecionada. Para esto vale la pena tener claro que los vecinos alrededor
 * de la celda son:
 * ???????????
 *
 * Viendo la celda X en una posicion general en el mapa:
 * ???????????
 *
 * Se obtiene asi entonces los vecinos de la celda activa.
 * */
uint8_t identify_cell_neighbours(Cell_map_t *grid_map, Cell_map_t *cell_to_check)
{
	uint8_t auxX = 0;
	uint8_t auxY = 0;
	uint8_t auxId = 0;
	// Vecino 1
	if (cell_to_check->pos_x > 0 && cell_to_check->pos_y > 0)
	{
		auxX = cell_to_check->pos_x / GRID_SIZE - 1;
		auxY = cell_to_check->pos_y / GRID_SIZE - 1;
		auxId = auxX % MAP_GRID_COLS + (auxY * MAP_GRID_COLS);
		cell_to_check->neighbors[0] = auxId;
	}
	// Vecino 2
	if (cell_to_check->pos_y > 0)
	{
		auxX = cell_to_check->pos_x / GRID_SIZE;
		auxY = cell_to_check->pos_y / GRID_SIZE - 1;
		auxId = auxX % MAP_GRID_COLS + (auxY * MAP_GRID_COLS);
		cell_to_check->neighbors[1] = auxId;
	}
	// Vecino 3
	if (cell_to_check->pos_y > 0 && cell_to_check->pos_x < GRID_SIZE * (MAP_GRID_COLS - 1))
	{
		auxX = cell_to_check->pos_x / GRID_SIZE + 1;
		auxY = cell_to_check->pos_y / GRID_SIZE - 1;
		auxId = auxX % MAP_GRID_COLS + (auxY * MAP_GRID_COLS);
		cell_to_check->neighbors[2] = auxId;
	}
	// Vecino 4
	if (cell_to_check->pos_x > 0)
	{
		auxX = cell_to_check->pos_x / GRID_SIZE - 1;
		auxY = cell_to_check->pos_y / GRID_SIZE;
		auxId = auxX % MAP_GRID_COLS + (auxY * MAP_GRID_COLS);
		cell_to_check->neighbors[3] = auxId;
	}
	// Vecino 5
	if (cell_to_check->pos_x < GRID_SIZE * (MAP_GRID_COLS - 1))
	{
		auxX = cell_to_check->pos_x / GRID_SIZE + 1;
		auxY = cell_to_check->pos_y / GRID_SIZE;
		auxId = auxX % MAP_GRID_COLS + (auxY * MAP_GRID_COLS);
		cell_to_check->neighbors[4] = auxId;
	}
	// Vecino 6
	if (cell_to_check->pos_x > 0 && cell_to_check->pos_y < GRID_SIZE * (MAP_GRID_ROWS - 1))
	{
		auxX = cell_to_check->pos_x / GRID_SIZE - 1;
		auxY = cell_to_check->pos_y / GRID_SIZE + 1;
		auxId = auxX % MAP_GRID_COLS + (auxY * MAP_GRID_COLS);
		cell_to_check->neighbors[5] = auxId;
	}
	// Vecino 7
	if (cell_to_check->pos_y < GRID_SIZE * (MAP_GRID_ROWS - 1))
	{
		auxX = cell_to_check->pos_x / GRID_SIZE;
		auxY = cell_to_check->pos_y / GRID_SIZE + 1;
		auxId = auxX % MAP_GRID_COLS + (auxY * MAP_GRID_COLS);
		cell_to_check->neighbors[6] = auxId;
	}
	// Vecino 8
	if (cell_to_check->pos_x < GRID_SIZE * (MAP_GRID_COLS - 1) && cell_to_check->pos_y < GRID_SIZE * (MAP_GRID_ROWS - 1))
	{
		auxX = cell_to_check->pos_x / GRID_SIZE + 1;
		auxY = cell_to_check->pos_y / GRID_SIZE + 1;
		auxId = auxX % MAP_GRID_COLS + (auxY * MAP_GRID_COLS);
		cell_to_check->neighbors[7] = auxId;
	}
}

/**
 * Esta funcion deberia utilizar la información entregada de alguna manera,
 * como por ejemplo un string con todos los datos o quizas en un JSON.
 *
 * En la primer idea se emplea el envio de datos linea a linea (string con los
 * caracteres de cada fila (row) indicando además cual es la liena que se desea
 * actualizar (0 a 7).
 * */
void populate_grid(char *row_data, uint8_t grid_row, Cell_map_t *grid_map)
{
	uint8_t aux = 0;
	uint8_t i = 0;

	while (row_data[i] != '\0')
	{
		if (row_data[i] == '.')
		{
			grid_map[grid_row * grid_cols + aux].type = FREE;
			aux++;
		}
		else if (row_data[i] == '#')
		{
			grid_map[grid_row * grid_cols + aux].type = OBSTACLE;
			aux++;
		}
		else if (row_data[i] == 'S')
		{
			if (ptr_start_cell)
			{
				printf("Multiples inicios\n");
				exit(EXIT_FAILURE);
			}
			grid_map[grid_row * grid_cols + aux].type = START;
			ptr_start_cell = &grid_map[grid_row * grid_cols + aux];
			aux++;
		}
		else if (row_data[i] == 'G')
		{
			if (ptr_goal_cell)
			{
				printf("Multiples objetivos\n");
				exit(EXIT_FAILURE);
			}
			grid_map[grid_row * grid_cols + aux].type = GOAL;
			ptr_goal_cell = &grid_map[grid_row * grid_cols + aux];
			aux++;
		}
		i++;
	}
}

/**
 * Esta funcion se encarga de crear un grid_map vacio, solo con los
 * elementos que corresponden creados por defecto (vacios, sin costos y
 * sin indicar de qué tipo son)
 */
void init_empty_grid_map(uint8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray)
{

	for (int i = 0; i < grid_rows; i++)
	{
		for (int j = 0; j < gridCols; j++)
		{
			cellArray[i * gridCols + j] = create_cell(j * GRID_SIZE, i * GRID_SIZE);
			cellArray[i * gridCols + j].id = i * gridCols + j;
		}
	}
}

/**
 * Me imrpime la información de una celda X, para verificar la informacion que ella contiene
 * y así poder mirar si todo esta funcionando correctamente.
 *
 * */
void print_single_cell_info(Cell_map_t *singleCell)
{

	printf("ID %d ", singleCell->id);
	printf("Pos X %d ", singleCell->pos_x);
	printf("Pos Y %d ", singleCell->pos_y);
	printf("Type %d ", singleCell->type);
	printf("G %d ", singleCell->g);
	printf("H %d ", singleCell->h);
	printf("F %d ", singleCell->f);
	printf("Parent %d ", singleCell->parent);
	printf("\n");
}

/**
 * Imprime la informacion de todas las celdas (en general del mapa)
 * */
void print_cells_info(Cell_map_t *cellArray)
{

	for (int i = 0; i < MAP_CELLS_COUNT; i++)
	{
		print_single_cell_info(&cellArray[i]);
	}
}

/**
 * Esta funcion se encarga de imprimir el mapa con sus caracteristicas
 * . default, empty cell
 * # obstacle
 * G Goal
 * S start
 * o Open
 * c Closed
 * El mapa se escribe fila a fila, por lo cual una idea es hacer fija la fila
 * e imprimir los elementos (columnas) de cada fila
 * */
void print_map(int8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray)
{

	for (int i = 0; i < gridRows; i++)
	{
		for (int j = 0; j < gridCols; j++)
		{
			uint8_t aux = cellArray[i * gridCols + j].type;
			switch (aux)
			{
			case FREE:
				if (cellArray[i * gridCols + j].state == NONE)
				{
					printf(". ");
				}
				else if (cellArray[i * gridCols + j].state == CLOSE)
				{
					printf("C ");
				}
				else if (cellArray[i * gridCols + j].state == OPEN)
				{
					printf("O ");
				}
				else if (cellArray[i * gridCols + j].state == PATH)
				{
					printf("* ");
				}

				break;
			case OBSTACLE:
				printf("# ");
				break;
			case START:
				printf("S ");
				break;
			case GOAL:
				printf("G ");
				break;
			default:
				break;
			}
		}
		printf("\n");
	}
}

/**
 * Esta funcion debe recibir como parametro el puntero a la ultima celda, que debe ser la "goal"
 * Con la información del ID_parent y entendiendo que todas las celdas visitadas
 * ya deben estar en el arreglo closed_list, se debe poder buscar la ruta y presentarla en pantalla
 *
 * */
void print_path(Cell_map_t *working_cell)
{

	printf("Path: ");
	while (working_cell->parent != 255)
	{
		if (working_cell != ptr_goal_cell)
		{
			working_cell->state = PATH;
		}
		printf("%d ", working_cell->id);
		working_cell = &grid_map_cells[working_cell->parent];
	}
	printf("\n");
	print_map(grid_cols, grid_rows, grid_map_cells);
}

/**
 * == A* algorithm ==
 * Aca es donde se hace toda la magia, todo lo de arriba es necesario, pero
 * el algoritmo se ejecuta es en esta funcion.
 *
 * Esta función es la descripción literal del pseudocodigo...
 * */
void A_star_algorithm(void)
{
	// 1. Encontrar la celda "Start" y "Goal"
	if (!get_cell_goal(grid_map_cells, grid_cols, grid_rows))
	{
		printf("Falta Objetivo \n");
		exit(EXIT_FAILURE);
	}
	if (!get_cell_start(grid_map_cells, grid_cols, grid_rows))
	{
		printf("Falta Inicio \n");
		exit(EXIT_FAILURE);
	}

	// 2. Incluir la celda "Start" en la lista open_list
	addTo_open_list(ptr_start_cell);

	// Loop
	while (1)
	{

		// 3. Seleccionar la celda con el menor valor F de la lista open_list
		ptr_current_cell = get_next_item();
		// 4. Remover de open list el current y añadir a close
		removeFrom_open_list(ptr_current_cell);

		addTo_closed_list(ptr_current_cell);
		if (ptr_current_cell == ptr_goal_cell)
		{
			// 5. Verificamos si la celda actual es la celda "Goal", entonces hemos terminado
			print_path(ptr_current_cell);
			break;
		}
		// 6. Buscamos los vecinos.
		identify_cell_neighbours(grid_map_cells, ptr_current_cell);
		// 7. Para cada vecino
		for (int i = 0; i < MAX_NEIGHBOURS; i++)
		{
			uint8_t auxNeigbhborID = ptr_current_cell->neighbors[i];
			if (auxNeigbhborID == 255)
			{
				continue;
			}
			Cell_map_t *auxCell = &grid_map_cells[auxNeigbhborID];
			// 8. Verificar si es atravesable o esta cerrada
			if (auxCell->type == OBSTACLE || auxCell->type == CLOSE)
			{
				continue;
			}
			// 9. Veririficamos si el nuevo F es más corto
			uint16_t auxGCost = get_G_cost(auxCell, ptr_current_cell);
			if (auxCell->state == NONE)
			{
				auxCell->g = auxGCost;
				auxCell->h = get_H_cost(ptr_goal_cell, auxCell);
				auxCell->f = get_F_cost(auxCell);
				auxCell->parent = ptr_current_cell->id;
				addTo_open_list(auxCell);
			}
			else if (auxCell->g > auxGCost && auxCell->state == OPEN)
			{
				auxCell->g = auxGCost;
				auxCell->f = get_F_cost(auxCell);
				auxCell->parent = ptr_current_cell->id;
				removeFrom_open_list(auxCell);
				addTo_open_list(auxCell);
			}
		}
	}
}
