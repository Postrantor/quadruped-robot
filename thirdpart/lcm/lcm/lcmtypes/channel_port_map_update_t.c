/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include "channel_port_map_update_t.h"

#include <string.h>

static int __channel_port_map_update_t_hash_computed;
static int64_t __channel_port_map_update_t_hash;

int64_t __channel_port_map_update_t_hash_recursive(const __lcm_hash_ptr *p) {
  const __lcm_hash_ptr *fp;
  for (fp = p; fp != NULL; fp = fp->parent)
    if (fp->v == __channel_port_map_update_t_get_hash) return 0;

  __lcm_hash_ptr cp;
  cp.parent = p;
  cp.v = (void *)__channel_port_map_update_t_get_hash;
  (void)cp;

  int64_t hash = 0x4216b98388375d0bLL + __int16_t_hash_recursive(&cp) + __int16_t_hash_recursive(&cp) +
                 __channel_to_port_t_hash_recursive(&cp);

  return (hash << 1) + ((hash >> 63) & 1);
}

int64_t __channel_port_map_update_t_get_hash(void) {
  if (!__channel_port_map_update_t_hash_computed) {
    __channel_port_map_update_t_hash = __channel_port_map_update_t_hash_recursive(NULL);
    __channel_port_map_update_t_hash_computed = 1;
  }

  return __channel_port_map_update_t_hash;
}

int __channel_port_map_update_t_encode_array(
    void *buf, int offset, int maxlen, const channel_port_map_update_t *p, int elements) {
  int pos = 0, thislen, element;

  for (element = 0; element < elements; element++) {
    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].num_ports), 1);
    if (thislen < 0)
      return thislen;
    else
      pos += thislen;

    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].num_channels), 1);
    if (thislen < 0)
      return thislen;
    else
      pos += thislen;

    thislen =
        __channel_to_port_t_encode_array(buf, offset + pos, maxlen - pos, p[element].mapping, p[element].num_channels);
    if (thislen < 0)
      return thislen;
    else
      pos += thislen;
  }
  return pos;
}

int channel_port_map_update_t_encode(void *buf, int offset, int maxlen, const channel_port_map_update_t *p) {
  int pos = 0, thislen;
  int64_t hash = __channel_port_map_update_t_get_hash();

  thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
  if (thislen < 0)
    return thislen;
  else
    pos += thislen;

  thislen = __channel_port_map_update_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
  if (thislen < 0)
    return thislen;
  else
    pos += thislen;

  return pos;
}

int __channel_port_map_update_t_encoded_array_size(const channel_port_map_update_t *p, int elements) {
  int size = 0, element;
  for (element = 0; element < elements; element++) {
    size += __int16_t_encoded_array_size(&(p[element].num_ports), 1);

    size += __int16_t_encoded_array_size(&(p[element].num_channels), 1);

    size += __channel_to_port_t_encoded_array_size(p[element].mapping, p[element].num_channels);
  }
  return size;
}

int channel_port_map_update_t_encoded_size(const channel_port_map_update_t *p) {
  return 8 + __channel_port_map_update_t_encoded_array_size(p, 1);
}

int __channel_port_map_update_t_decode_array(
    const void *buf, int offset, int maxlen, channel_port_map_update_t *p, int elements) {
  int pos = 0, thislen, element;

  for (element = 0; element < elements; element++) {
    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].num_ports), 1);
    if (thislen < 0)
      return thislen;
    else
      pos += thislen;

    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].num_channels), 1);
    if (thislen < 0)
      return thislen;
    else
      pos += thislen;

    p[element].mapping = (channel_to_port_t *)lcm_malloc(sizeof(channel_to_port_t) * p[element].num_channels);
    thislen =
        __channel_to_port_t_decode_array(buf, offset + pos, maxlen - pos, p[element].mapping, p[element].num_channels);
    if (thislen < 0)
      return thislen;
    else
      pos += thislen;
  }
  return pos;
}

int __channel_port_map_update_t_decode_array_cleanup(channel_port_map_update_t *p, int elements) {
  int element;
  for (element = 0; element < elements; element++) {
    __int16_t_decode_array_cleanup(&(p[element].num_ports), 1);

    __int16_t_decode_array_cleanup(&(p[element].num_channels), 1);

    __channel_to_port_t_decode_array_cleanup(p[element].mapping, p[element].num_channels);
    if (p[element].mapping) free(p[element].mapping);
  }
  return 0;
}

int channel_port_map_update_t_decode(const void *buf, int offset, int maxlen, channel_port_map_update_t *p) {
  int pos = 0, thislen;
  int64_t hash = __channel_port_map_update_t_get_hash();

  int64_t this_hash;
  thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
  if (thislen < 0)
    return thislen;
  else
    pos += thislen;
  if (this_hash != hash) return -1;

  thislen = __channel_port_map_update_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
  if (thislen < 0)
    return thislen;
  else
    pos += thislen;

  return pos;
}

int channel_port_map_update_t_decode_cleanup(channel_port_map_update_t *p) {
  return __channel_port_map_update_t_decode_array_cleanup(p, 1);
}

int __channel_port_map_update_t_clone_array(
    const channel_port_map_update_t *p, channel_port_map_update_t *q, int elements) {
  int element;
  for (element = 0; element < elements; element++) {
    __int16_t_clone_array(&(p[element].num_ports), &(q[element].num_ports), 1);

    __int16_t_clone_array(&(p[element].num_channels), &(q[element].num_channels), 1);

    q[element].mapping = (channel_to_port_t *)lcm_malloc(sizeof(channel_to_port_t) * q[element].num_channels);
    __channel_to_port_t_clone_array(p[element].mapping, q[element].mapping, p[element].num_channels);
  }
  return 0;
}

channel_port_map_update_t *channel_port_map_update_t_copy(const channel_port_map_update_t *p) {
  channel_port_map_update_t *q = (channel_port_map_update_t *)malloc(sizeof(channel_port_map_update_t));
  __channel_port_map_update_t_clone_array(p, q, 1);
  return q;
}

void channel_port_map_update_t_destroy(channel_port_map_update_t *p) {
  __channel_port_map_update_t_decode_array_cleanup(p, 1);
  free(p);
}