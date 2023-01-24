/*
 * list
 *
 *  Created on: Jan 17, 2023
 *      Author: marijn
 */


/// header guard
#ifndef INC_LIST_H_
#define INC_LIST_H_


/// includes
#include <stdint.h>
#include <stdlib.h>


/// types
typedef struct {
	void* data;
	void* next;
} Node_TypeDef;

typedef struct {
	Node_TypeDef*	start;
	Node_TypeDef*	end;
	uint32_t		size;
} Linked_List;


// functions
Linked_List* new_list();
void delete_list(Linked_List* list);

void add(Linked_List* list, void* data);						// add node to the end of the list
void push(Linked_List* list, void* data);						// add node to the start of the list
void insert(Linked_List* list, uint32_t index, void* data);		// add node to the index in the list
void pop(Linked_List* list);									// remove last node
void delete(Linked_List* list, uint32_t index);					// delete node at index

Node_TypeDef* get(Linked_List* list, uint32_t index);

#endif /* INC_LIST_H_ */
