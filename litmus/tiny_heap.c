/* 
 * 	tiny_heap.c
 */
//#include <litmus/rt_param.h>
//#include <litmus/tiny_heap.h>

inline void t_h_init(struct tiny_heap *h) {
	
	int i;
	
	BUG_ON(!h);
	
	for(i = 0; i < TINY_HEAP_SIZE; i++) 
	{
		h->data[i] = ULLONG_MAX;
	}
	
	h->elements = -1;

}

inline unsigned long long t_h_get_min(struct tiny_heap *h) {
	
	if (h->elements < 0)
		return 0;
		
	return h->data[0];
}

inline void t_h_min_heapify(struct tiny_heap *h, int i)
{
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

inline unsigned long long t_h_extract_min(struct tiny_heap *h) {
	
	unsigned long long min;
	
	if (h->elements < 0)
		return 0;	
	min = h->data[0];	
	h->data[0] = h->data[h->elements];
	h->elements -= 1;
	t_h_min_heapify(h,0);
	return min;
}

inline int t_h_is_empty(struct tiny_heap *h) {
	return (h->elements < 0);
}

inline int t_h_insert(struct tiny_heap *h, unsigned long long value) {
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

