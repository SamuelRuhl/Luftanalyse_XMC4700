
#ifndef SPI_H
#define SPI_H

/* FT800 low-level functions */
//void SPI_init(void);			/* SPI1 init for FT800 */
//void SPI_speedup(void);			/* Speed Up SPI1 */
uint8_t SPI_send(uint8_t data);		/* Send char to SPI1 */
uint8_t SPI_rec(void);		/* Receive char from SPI1 */

void FT_spi_select(void);		/* Select FT800 */
void FT_spi_deselect(void);		/* Deselect FT800 */

void us_delay(uint32_t);
void ms_delay(uint32_t);

#endif
