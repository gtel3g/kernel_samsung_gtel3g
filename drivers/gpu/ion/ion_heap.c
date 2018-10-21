/*
 * drivers/gpu/ion/ion_heap.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/ion.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/rtmutex.h>
#include <linux/sched.h>
#include <linux/scatterlist.h>
#include <linux/vmalloc.h>
#include "ion_priv.h"

void *ion_heap_map_kernel(struct ion_heap *heap,
			  struct ion_buffer *buffer)
{
	struct scatterlist *sg;
	int i, j;
	void *vaddr;
	pgprot_t pgprot;
	struct sg_table *table = buffer->sg_table;
	int npages = PAGE_ALIGN(buffer->size) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;

	if (!pages)
		return 0;

	if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	for_each_sg(table->sgl, sg, table->nents, i) {
		int npages_this_entry = PAGE_ALIGN(sg_dma_len(sg)) / PAGE_SIZE;
		struct page *page = sg_page(sg);
		BUG_ON(i >= npages);
		for (j = 0; j < npages_this_entry; j++) {
			*(tmp++) = page++;
		}
	}
	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);

	if (vaddr == NULL)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

void ion_heap_unmap_kernel(struct ion_heap *heap,
			   struct ion_buffer *buffer)
{
	vunmap(buffer->vaddr);
}

int ion_heap_map_user(struct ion_heap *heap, struct ion_buffer *buffer,
		      struct vm_area_struct *vma)
{
	struct sg_table *table = buffer->sg_table;
	unsigned long addr = vma->vm_start;
	unsigned long offset = vma->vm_pgoff * PAGE_SIZE;
	struct scatterlist *sg;
	int i;

	for_each_sg(table->sgl, sg, table->nents, i) {
		struct page *page = sg_page(sg);
		unsigned long remainder = vma->vm_end - addr;
		unsigned long len = sg_dma_len(sg);

		if (offset >= sg_dma_len(sg)) {
			offset -= sg_dma_len(sg);
			continue;
		} else if (offset) {
			page += offset / PAGE_SIZE;
			len = sg_dma_len(sg) - offset;
			offset = 0;
		}
		len = min(len, remainder);
		remap_pfn_range(vma, addr, page_to_pfn(page), len,
				vma->vm_page_prot);
		addr += len;
		if (addr >= vma->vm_end)
			return 0;
	}
	return 0;
}

int ion_heap_buffer_zero(struct ion_buffer *buffer)
{
	struct sg_table *table = buffer->sg_table;
	pgprot_t pgprot;
	struct scatterlist *sg;
	struct vm_struct *vm_struct;
	int i, j, ret = 0;

	if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	vm_struct = get_vm_area(PAGE_SIZE, VM_ALLOC);
	if (!vm_struct)
		return -ENOMEM;

	for_each_sg(table->sgl, sg, table->nents, i) {
		struct page *page = sg_page(sg);
		unsigned long len = sg_dma_len(sg);

		for (j = 0; j < len / PAGE_SIZE; j++) {
			struct page *sub_page = page + j;
			struct page **pages = &sub_page;
			ret = map_vm_area(vm_struct, pgprot, pages);
			if (ret)
				goto end;
			memset(vm_struct->addr, 0, PAGE_SIZE);
			unmap_kernel_range((unsigned long)vm_struct->addr,
					   PAGE_SIZE);
		}
	}
end:
	free_vm_area(vm_struct);
	return ret;
}

struct page *ion_heap_alloc_pages(struct ion_buffer *buffer, gfp_t gfp_flags,
				  unsigned int order)
{
	struct page *page = alloc_pages(gfp_flags, order);

	if (!page)
		return page;

	if (ion_buffer_fault_user_mappings(buffer))
		split_page(page, order);

	return page;
}

void ion_heap_free_pages(struct ion_buffer *buffer, struct page *page,
			 unsigned int order)
{
	int i;

	if (!ion_buffer_fault_user_mappings(buffer)) {
		__free_pages(page, order);
		return;
	}
	for (i = 0; i < (1 << order); i++)
		__free_page(page + i);
}

void ion_heap_freelist_add(struct ion_heap *heap, struct ion_buffer * buffer)
{
	spin_lock(&heap->free_lock);
	list_add(&buffer->list, &heap->free_list);
	heap->free_list_size += buffer->size;
	spin_unlock(&heap->free_lock);
	wake_up(&heap->waitqueue);
}

size_t ion_heap_freelist_size(struct ion_heap *heap)
{
	size_t size;

	spin_lock(&heap->free_lock);
	size = heap->free_list_size;
	spin_unlock(&heap->free_lock);

	return size;
}

static size_t _ion_heap_freelist_drain(struct ion_heap *heap, int cached, size_t size,
                        bool skip_pools)
{
	struct ion_buffer *buffer, *tmp;
	size_t total_drained = 0;
	struct list_head free_list;

	if (ion_heap_freelist_size(heap) == 0)
		return 0;

	spin_lock(&heap->free_lock);
	if (size == 0)
		size = heap->free_list_size;

        INIT_LIST_HEAD(&free_list);

	list_for_each_entry_safe(buffer, tmp, &heap->free_list, list) {
		if (total_drained >= size)
			break;
		if (!(cached < 0 || cached == ion_buffer_cached(buffer)))
			continue;
		list_del(&buffer->list);
            list_add(&buffer->list, &free_list);
		heap->free_list_size -= buffer->size;
            if (skip_pools)
                buffer->private_flags |= ION_PRIV_FLAG_SHRINKER_FREE;
		total_drained += buffer->size;
        }
        spin_unlock(&heap->free_lock);

        list_for_each_entry_safe(buffer, tmp, &free_list, list) {
            list_del(&buffer->list);
		ion_buffer_destroy(buffer);
	}

	return total_drained;
}

size_t ion_heap_freelist_drain(struct ion_heap *heap, int cached, size_t size)
{
	return _ion_heap_freelist_drain(heap, cached, size, false);
}

size_t ion_heap_freelist_shrink(struct ion_heap *heap, int cached, size_t size)
{
	return _ion_heap_freelist_drain(heap, cached, size, true);
}


int ion_heap_deferred_free(void *data)
{
	struct ion_heap *heap = data;

	while (true) {
		struct ion_buffer *buffer;

		wait_event_freezable(heap->waitqueue,
				     ion_heap_freelist_size(heap) > 0);

		spin_lock(&heap->free_lock);
		if (list_empty(&heap->free_list)) {
			/*
			 *  Sprd Change
			 *  Add a protect to avoid the thread waked up allways
			 *  when free_list_size is overwrited by abnormal operation.
			 * */
			if (heap->free_list_size > 0)
			{
			    printk(KERN_INFO "ion buffer free_list_size:%u is in abnormal state, so do reset\n",
					    (unsigned int)heap->free_list_size);
			    heap->free_list_size = 0;
			}
			spin_unlock(&heap->free_lock);
			continue;
		}
		buffer = list_first_entry(&heap->free_list, struct ion_buffer,
					  list);
		list_del(&buffer->list);
		heap->free_list_size -= buffer->size;
		spin_unlock(&heap->free_lock);
		ion_buffer_destroy(buffer);
	}

	return 0;
}

int ion_heap_init_deferred_free(struct ion_heap *heap)
{
	struct sched_param param = { .sched_priority = 0 };

	INIT_LIST_HEAD(&heap->free_list);
	heap->free_list_size = 0;
	spin_lock_init(&heap->free_lock);
	init_waitqueue_head(&heap->waitqueue);
	heap->task = kthread_run(ion_heap_deferred_free, heap,
				 "%s", heap->name);
	sched_setscheduler(heap->task, SCHED_IDLE, &param);
	if (IS_ERR(heap->task)) {
		pr_err("%s: creating thread for deferred free failed\n",
		       __func__);
		return PTR_RET(heap->task);
	}
	return 0;
}

struct ion_heap *ion_heap_create(struct ion_platform_heap *heap_data)
{
	struct ion_heap *heap = NULL;

	switch (heap_data->type) {
	case ION_HEAP_TYPE_SYSTEM_CONTIG:
		pr_err("%s: Heap type is disabled: %d\n", __func__,
		       heap_data->type);
		return ERR_PTR(-EINVAL);
	case ION_HEAP_TYPE_SYSTEM:
		heap = ion_system_heap_create(heap_data);
		break;
	case ION_HEAP_TYPE_CARVEOUT:
		heap = ion_carveout_heap_create(heap_data);
		break;
	case ION_HEAP_TYPE_CHUNK:
		heap = ion_chunk_heap_create(heap_data);
		break;
	case ION_HEAP_TYPE_DMA:
		heap = ion_cma_heap_create(heap_data, NULL);
		break;
	default:
		pr_err("%s: Invalid heap type %d\n", __func__,
		       heap_data->type);
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR_OR_NULL(heap)) {
		pr_err("%s: error creating heap %s type %d base %lu size %u\n",
		       __func__, heap_data->name, heap_data->type,
		       heap_data->base, heap_data->size);
		return ERR_PTR(-EINVAL);
	}

	heap->name = heap_data->name;
	heap->id = heap_data->id;
	return heap;
}

void ion_heap_destroy(struct ion_heap *heap)
{
	if (!heap)
		return;

	switch (heap->type) {
	case ION_HEAP_TYPE_SYSTEM_CONTIG:
		pr_err("%s: Heap type is disabled: %d\n", __func__,
		       heap->type);
		break;
	case ION_HEAP_TYPE_SYSTEM:
		ion_system_heap_destroy(heap);
		break;
	case ION_HEAP_TYPE_CARVEOUT:
		ion_carveout_heap_destroy(heap);
		break;
	case ION_HEAP_TYPE_CHUNK:
		ion_chunk_heap_destroy(heap);
		break;
	case ION_HEAP_TYPE_DMA:
		ion_cma_heap_destroy(heap);
		break;
	default:
		pr_err("%s: Invalid heap type %d\n", __func__,
		       heap->type);
	}
}
