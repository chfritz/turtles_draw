/*
 Parson ( http://kgabis.github.com/parson/ )
 Copyright (c) 2012 - 2014 Krzysztof Gabis
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/

#ifndef parson_parson_h
#define parson_parson_h

#ifdef __cplusplus
extern "C"
{
#endif    
    
#include <stddef.h>   /* size_t */    
    
/* Types and enums */
typedef struct parson_json_object_t JSON_Object;
typedef struct parson_json_array_t  JSON_Array;
typedef struct parson_json_value_t  JSON_Value;

enum parson_json_value_type {
    JSONError   = -1,
    JSONNull    = 1,
    JSONString  = 2,
    JSONNumber  = 3,
    JSONObject  = 4,
    JSONArray   = 5,
    JSONBoolean = 6
};
typedef int JSON_Value_Type;
    
enum parson_json_result_t {
    JSONSuccess = 0,
    JSONFailure = -1
};
typedef int JSON_Status;
   
/* Parses first JSON value in a file, returns NULL in case of error */
JSON_Value * parson_json_parse_file(const char *filename);

/* Parses first JSON value in a file and ignores comments (/ * * / and //),
   returns NULL in case of error */
JSON_Value * parson_json_parse_file_with_comments(const char *filename);
    
/*  Parses first JSON value in a string, returns NULL in case of error */
JSON_Value * parson_json_parse_string(const char *string);

/*  Parses first JSON value in a string and ignores comments (/ * * / and //),
    returns NULL in case of error */
JSON_Value * parson_json_parse_string_with_comments(const char *string);
    
/* Serialization */
size_t      parson_json_serialization_size(const JSON_Value *value);
JSON_Status parson_json_serialize_to_buffer(const JSON_Value *value, char *buf, size_t buf_size_in_bytes);
JSON_Status parson_json_serialize_to_file(const JSON_Value *value, const char *filename);
char *      parson_json_serialize_to_string(const JSON_Value *value);
void        parson_json_free_serialized_string(char *string); /* frees string from parson_json_serialize_to_string */

/* Comparing */
int  parson_json_value_equals(const JSON_Value *a, const JSON_Value *b);
    
/* Validation
   This is *NOT* JSON Schema. It validates json by checking if object have identically 
   named fields with matching types.
   For example schema {"name":"", "age":0} will validate 
   {"name":"Joe", "age":25} and {"name":"Joe", "age":25, "gender":"m"},
   but not {"name":"Joe"} or {"name":"Joe", "age":"Cucumber"}.
   In case of arrays, only first value in schema is checked against all values in tested array.
   Empty objects ({}) validate all objects, empty arrays ([]) validate all arrays,
   null validates values of every type.
 */
JSON_Status parson_json_validate(const JSON_Value *schema, const JSON_Value *value);
    
/*
 * JSON Object
 */
JSON_Value  * parson_json_object_get_value  (const JSON_Object *object, const char *name);
const char  * parson_json_object_get_string (const JSON_Object *object, const char *name);
JSON_Object * parson_json_object_get_object (const JSON_Object *object, const char *name);
JSON_Array  * parson_json_object_get_array  (const JSON_Object *object, const char *name);
double        parson_json_object_get_number (const JSON_Object *object, const char *name); /* returns 0 on fail */
int           parson_json_object_get_boolean(const JSON_Object *object, const char *name); /* returns -1 on fail */

/* dotget functions enable addressing values with dot notation in nested objects,
 just like in structs or c++/java/c# objects (e.g. objectA.objectB.value).
 Because valid names in JSON can contain dots, some values may be inaccessible
 this way. */
JSON_Value  * parson_json_object_dotget_value  (const JSON_Object *object, const char *name);
const char  * parson_json_object_dotget_string (const JSON_Object *object, const char *name);
JSON_Object * parson_json_object_dotget_object (const JSON_Object *object, const char *name);
JSON_Array  * parson_json_object_dotget_array  (const JSON_Object *object, const char *name);
double        parson_json_object_dotget_number (const JSON_Object *object, const char *name); /* returns 0 on fail */
int           parson_json_object_dotget_boolean(const JSON_Object *object, const char *name); /* returns -1 on fail */

/* Functions to get available names */
size_t        parson_json_object_get_count(const JSON_Object *object);
const char  * parson_json_object_get_name (const JSON_Object *object, size_t index);
    
/* Creates new name-value pair or frees and replaces old value with new one. */
JSON_Status parson_json_object_set_value(JSON_Object *object, const char *name, JSON_Value *value);
JSON_Status parson_json_object_set_string(JSON_Object *object, const char *name, const char *string);
JSON_Status parson_json_object_set_number(JSON_Object *object, const char *name, double number);
JSON_Status parson_json_object_set_boolean(JSON_Object *object, const char *name, int boolean);
JSON_Status parson_json_object_set_null(JSON_Object *object, const char *name);

/* Works like dotget functions, but creates whole hierarchy if necessary. */
JSON_Status parson_json_object_dotset_value(JSON_Object *object, const char *name, JSON_Value *value);
JSON_Status parson_json_object_dotset_string(JSON_Object *object, const char *name, const char *string);
JSON_Status parson_json_object_dotset_number(JSON_Object *object, const char *name, double number);
JSON_Status parson_json_object_dotset_boolean(JSON_Object *object, const char *name, int boolean);
JSON_Status parson_json_object_dotset_null(JSON_Object *object, const char *name);

/* Frees and removes name-value pair */
JSON_Status parson_json_object_remove(JSON_Object *object, const char *name);

/* Works like dotget function, but removes name-value pair only on exact match. */
JSON_Status parson_json_object_dotremove(JSON_Object *object, const char *key);

/* Removes all name-value pairs in object */
JSON_Status parson_json_object_clear(JSON_Object *object);
    
/* 
 *JSON Array 
 */
JSON_Value  * parson_json_array_get_value  (const JSON_Array *array, size_t index);
const char  * parson_json_array_get_string (const JSON_Array *array, size_t index);
JSON_Object * parson_json_array_get_object (const JSON_Array *array, size_t index);
JSON_Array  * parson_json_array_get_array  (const JSON_Array *array, size_t index);
double        parson_json_array_get_number (const JSON_Array *array, size_t index); /* returns 0 on fail */
int           parson_json_array_get_boolean(const JSON_Array *array, size_t index); /* returns -1 on fail */
size_t        parson_json_array_get_count  (const JSON_Array *array);
    
/* Frees and removes value at given index, does nothing and returns JSONFailure if index doesn't exist.
 * Order of values in array may change during execution.  */
JSON_Status parson_json_array_remove(JSON_Array *array, size_t i);

/* Frees and removes from array value at given index and replaces it with given one.
 * Does nothing and returns JSONFailure if index doesn't exist. */
JSON_Status parson_json_array_replace_value(JSON_Array *array, size_t i, JSON_Value *value);
JSON_Status parson_json_array_replace_string(JSON_Array *array, size_t i, const char* string);
JSON_Status parson_json_array_replace_number(JSON_Array *array, size_t i, double number);
JSON_Status parson_json_array_replace_boolean(JSON_Array *array, size_t i, int boolean);
JSON_Status parson_json_array_replace_null(JSON_Array *array, size_t i);

/* Frees and removes all values from array */
JSON_Status parson_json_array_clear(JSON_Array *array);

/* Appends new value at the end of array. */
JSON_Status parson_json_array_append_value(JSON_Array *array, JSON_Value *value);
JSON_Status parson_json_array_append_string(JSON_Array *array, const char *string);
JSON_Status parson_json_array_append_number(JSON_Array *array, double number);
JSON_Status parson_json_array_append_boolean(JSON_Array *array, int boolean);
JSON_Status parson_json_array_append_null(JSON_Array *array);
    
/*
 *JSON Value
 */
JSON_Value * parson_json_value_init_object (void);
JSON_Value * parson_json_value_init_array  (void);
JSON_Value * parson_json_value_init_string (const char *string); /* copies passed string */
JSON_Value * parson_json_value_init_number (double number);
JSON_Value * parson_json_value_init_boolean(int boolean);
JSON_Value * parson_json_value_init_null   (void);
JSON_Value * parson_json_value_deep_copy   (const JSON_Value *value);
void         parson_json_value_free        (JSON_Value *value);

JSON_Value_Type parson_json_value_get_type   (const JSON_Value *value);
JSON_Object *   parson_json_value_get_object (const JSON_Value *value);
JSON_Array  *   parson_json_value_get_array  (const JSON_Value *value);
const char  *   parson_json_value_get_string (const JSON_Value *value);
double          parson_json_value_get_number (const JSON_Value *value);
int             parson_json_value_get_boolean(const JSON_Value *value);

/* Same as above, but shorter */
JSON_Value_Type parson_json_type   (const JSON_Value *value);
JSON_Object *   parson_json_object (const JSON_Value *value);
JSON_Array  *   parson_json_array  (const JSON_Value *value);
const char  *   parson_json_string (const JSON_Value *value);
double          parson_json_number (const JSON_Value *value);
int             parson_json_boolean(const JSON_Value *value);
    
#ifdef __cplusplus
}
#endif

#endif
