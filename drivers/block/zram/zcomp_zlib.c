/*
 * Copyright (C) 2015 Sergey Senozhatsky.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/zlib.h>

#include "zcomp_zlib.h"

#define ZLIB_COMPRESSION_LEVEL	3

static void *zlib_create(void)
{
	z_stream *stream;
	size_t size;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return NULL;

	size = max(zlib_deflate_workspacesize(MAX_WBITS, MAX_MEM_LEVEL),
			zlib_inflate_workspacesize());
	stream->workspace = vmalloc(size);
	if (!stream->workspace) {
		kfree(stream);
		stream = NULL;
	}

	return stream;
}

static void zlib_destroy(void *private)
{
	z_stream *stream = private;

	vfree(stream->workspace);
	kfree(stream);
}

static int zlib_flags(void)
{
	return ZCOMP_NEED_READ_ZSTRM;
}

static int zlib_compress(const unsigned char *src, unsigned char *dst,
		size_t *dst_len, void *private)
{
	z_stream *stream = private;
	int err;

	err = zlib_deflateInit(stream, ZLIB_COMPRESSION_LEVEL);
	if (err != Z_OK)
		goto out;

	stream->next_in = src;
	stream->avail_in = PAGE_SIZE;
	stream->total_in = 0;
	stream->next_out = dst;
	stream->avail_out = PAGE_SIZE;
	stream->total_out = 0;

	err = zlib_deflate(stream, Z_FINISH);
	if (err != Z_STREAM_END)
		goto out;

	err = zlib_deflateEnd(stream);
	if (err != Z_OK)
		goto out;

	if (stream->total_out >= stream->total_in)
		goto out;

	*dst_len = stream->total_out;
out:
	return err == Z_OK ? 0 : err;
}

static int zlib_decompress(const unsigned char *src, size_t src_len,
		unsigned char *dst, void *private)
{
	z_stream *stream = private;
	int err;

	err = zlib_inflateInit(stream);
	if (err != Z_OK)
		goto out;

	stream->next_in = src;
	stream->avail_in = src_len;
	stream->total_in = 0;
	stream->next_out = dst;
	stream->avail_out = PAGE_SIZE;
	stream->total_out = 0;

	err = zlib_inflate(stream, Z_FINISH);
	if (err != Z_STREAM_END)
		goto out;

	err = zlib_inflateEnd(stream);
	if (err != Z_OK)
		goto out;
out:
	return err == Z_OK ? 0 : err;
}

struct zcomp_backend zcomp_zlib = {
	.compress = zlib_compress,
	.decompress = zlib_decompress,
	.create = zlib_create,
	.destroy = zlib_destroy,
	.flags = zlib_flags,
	.name = "zlib",
};
