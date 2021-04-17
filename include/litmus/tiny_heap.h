/* 
 * 	tiny_heap.h
 */
#ifndef TINY_HEAP
#define TINY_HEAP

#include <linux/kernel.h>

#define TINY_HEAP_SIZE 	4

#define parent(i) 	(i / 2)		//(i>>1)
#define left(i)		(2 * i)		//(i<<1)
#define right(i)	(2 * i) + 1	//(i<<1) + 1

struct tiny_elem;

struct tiny_heap {
	tiny_elem		*data[TINY_HEAP_SIZE];
	int			elements;
};

void t_h_init(struct tiny_heap *h){
	
	int i;
	
	BUG_ON(!h);
	
	for(i = 0; i < TINY_HEAP_SIZE; i++) 
	{
		h->data[i] = LLONG_MAX;
	}
	
	h->elements = -1;

}

unsigned long long t_h_get_min(struct tiny_heap *h){
	
	if (h->elements < 0)
		return 0;
		
	return h->data[0];
}

void t_h_min_heapify(struct tiny_heap *h, int i){
	int l, r, largest;
	unsigned long long tmp;
			
	// left = 2*i
	// right = 2*i + 1
	while (1) {
		l = left(i);
		r = right(i);
		if (l <= h->elements && h->data[l] < h->data[i])
			largest = l;
		else largest = i;
		if (r <= h->elements && h->data[r] < h->data[largest])
			largest = r;
		if (largest != i) {
			tmp = h->data[largest];
			h->data[largest] = h->data[i];
			h->data[i] = tmp;
			i = largest;
		} else 
			return;
	}
	
}

unsigned long long t_h_extract_min(struct tiny_heap *h){
	
	unsigned long long min;
	
	if (h->elements < 0)
		return 0;	
	min = h->data[0];	
	h->data[0] = h->data[h->elements];
	h->elements -= 1;
	t_h_min_heapify(h,0);
	return min;
}

int t_h_is_empty(struct tiny_heap *h){
	return (h->elements < 0);
}

int t_h_insert(struct tiny_heap *h, unsigned long long value){
	int i, parent;
	unsigned long long tmp;
	
	h->elements += 1;
	i = h->elements;
	h->data[i] = value;
	parent = parent(i);
	while (i > 0 && h->data[i] < h->data[parent]) {
		tmp = h->data[parent];
		h->data[parent] = h->data[i];
		h->data[i] = tmp;
		i = parent;
		parent = parent(i);
	}
	return 1;
}
#endif
