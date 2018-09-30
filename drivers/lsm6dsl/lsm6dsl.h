/**
 * \file lsm6dsl.h
 *
 * \author Mav Cuyugan
 * \brief  LSM6DSL driver header
 **/

#ifndef _LSM6DSL_H
#define _LSM6DSL_H

typedef enum
{
  LSM6DSL_STATE_STOP = 0,
  LSM6DSL_STATE_START,
  LSM6DSL_STATE_LOWPOWER
} lsm6dsl_state_t;

typedef enum
{
  LSM6DSL_OK = 0,
  LSM6DSL_ERROR,
} lsm6dsl_status_t;

typedef struct
{

} LSM6DSLConfig;

typedef struct
{
  const LSM6DSLConfig* cfg;
  lsm6dsl_state_t state;
} LSM6DSLHandle;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initialize LSM6DSL handle object
 *
 * \param[out] handle - LSM6DSL handle
 **/
void lsm6dslObjectInit(LSM6DSLHandle* handle);

/**
 * \brief Start LSM6DSL device
 *
 * \param[out] handle - LSM6DSL handle
 * \param[in]  cfg    - driver configurations
 *
 * \return Driver status
 **/
lsm6dsl_status_t lsm6dslStart(LSM6DSLHandle* handle, const LSM6DSLConfig* cfg);

#ifdef __cplusplus
}
#endif

#endif /* _LSM6DSL_H */
