/*
 * list.c
 *
 *  Created on: Jan 17, 2023
 *      Author: marijn
 */
#include "list.h"


// hidden
void delete_chain(Node_TypeDef* node) {
	if (!node) { return; }
	if (node->next) { delete_chain(node->next);	}
	free(node->data);
	free(node);
}
void init(Linked_List* list) {
	// delete the chain from index 1
	if (list->start && list->start->next) { delete_chain(list->start->next); }
	list->start = calloc(1, sizeof(Node_TypeDef));
	list->size = 1;
	list->end = list->start;
}


// exposed
Linked_List* new_list()				{ return (Linked_List*)calloc(1, sizeof(Linked_List)); }
void delete_list(Linked_List* list)	{ delete_chain(list->start); free(list); }

void add(Linked_List* list, void* data) {
	if (list->size == 0) { init(list); list->start->data = data; return; }
	Node_TypeDef* node = calloc(1, sizeof(Node_TypeDef));
	node->data = data;
	list->end->next = node;
	list->end = node;
	list->size++;
}
void push(Linked_List* list, void* data) {
	if (list->size == 0) { init(list); list->start->data = data; return; }
	Node_TypeDef* node = calloc(1, sizeof(Node_TypeDef));
	node->data = data;
	node->next = list->start;
	list->start = node;
	list->size++;
}
void insert(Linked_List* list, uint32_t index, void* data) {
	if (list->size == 0) { init(list); list->start->data = data; return; }
	if (index >= list->size) { return; }
	Node_TypeDef* new_node = calloc(1, sizeof(Node_TypeDef));
	new_node->data = data;
	Node_TypeDef* node = get(list, index);
	new_node->next = node->next;
	node->next = new_node;
	list->end = get(list, list->size - 1);  // reastablish end node just to be sure
	list->size++;
}
void pop(Linked_List* list) {
	if (list->size == 0) { return; }
	free(list->end->data);
	free(list->end);
	list->end = get(list, list->size - 2);
}
void delete(Linked_List* list, uint32_t index){
	if (list->size == 0) { return; }
	Node_TypeDef* node = get(list, index - 1);
	Node_TypeDef* del_node = node->next;
	node->next = del_node->next;
	free(del_node->data);
	free(del_node);
}

Node_TypeDef* get(Linked_List* list, uint32_t index) {
	if (!list->start) { return (void*)0; }
	Node_TypeDef* node = list->start;
	for (uint32_t i = 1; i < index; i++) { if (!node->next) { return node; } node = node->next; }
	return node;
}
