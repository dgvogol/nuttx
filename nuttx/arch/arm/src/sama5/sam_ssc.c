/****************************************************************************
 * arch/arm/src/sama5/sam_ssc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <wdog.h>
#include <errno.h>
#include <assert.h>
#include <queue.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/audio/i2s.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "sam_pio.h"
#include "sam_dmac.h"
#include "sam_memories.h"
#include "sam_periphclks.h"
#include "sam_ssc.h"
#include "chip/sam_pmc.h"
#include "chip/sam_ssc.h"
#include "chip/sam_pinmap.h"

#if defined(CONFIG_SAMA5_SSC0) || defined(CONFIG_SAMA5_SSC1)

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef SAMA5_SSC_MAXINFLIGHT
#  define SAMA5_SSC_MAXINFLIGHT 16
#endif

#if defined(CONFIG_SAMA5_SSC0) && !defined(CONFIG_SAMA5_DMAC0)
#  error CONFIG_SAMA5_DMAC0 required by SSC0
#endif

#if defined(CONFIG_SAMA5_SSC1) && !defined(CONFIG_SAMA5_DMAC1)
#  error CONFIG_SAMA5_DMAC1 required by SSC1
#endif

#ifndef CONFIG_SAMA5_SSC0_DATALEN
#  define CONFIG_SAMA5_SSC0_DATALEN 16
#endif

#if CONFIG_SAMA5_SSC0_DATALEN < 2 || CONFIG_SAMA5_SSC0_DATALEN > 32
#  error Invalid value for CONFIG_SAMA5_SSC0_DATALEN
#endif

/* Check if we need to build RX and/or TX support */

#undef SSC_HAVE_RX
#undef SSC_HAVE_TX

#if (defined(CONFIG_SAMA5_SSC0) && defined(CONFIG_SAMA5_SSC0_RX)) || \
    (defined(CONFIG_SAMA5_SSC1) && defined(CONFIG_SAMA5_SSC1_RX))
#  define SSC_HAVE_RX
#endif

#if (defined(CONFIG_SAMA5_SSC0) && defined(CONFIG_SAMA5_SSC0_TX)) || \
    (defined(CONFIG_SAMA5_SSC1) && defined(CONFIG_SAMA5_SSC1_TX))
#  define SSC_HAVE_TX
#endif

#define SSC_DATNB  (1) /*  Data number per frame */

/* Clocking *****************************************************************/
/* Select MCU-specific settings
 *
 * SSC is driven by the main clock, divided down so that the maximum
 * peripheral clocking is not exceeded.
 */

#if BOARD_MCK_FREQUENCY <= SAM_SSC_MAXPERCLK
#  define SSC_FREQUENCY BOARD_MCK_FREQUENCY
#  define SSC_PCR_DIV PMC_PCR_DIV1
#elif (BOARD_MCK_FREQUENCY >> 1) <= SAM_SSC_MAXPERCLK
#  define SSC_FREQUENCY (BOARD_MCK_FREQUENCY >> 1)
#  define SSC_PCR_DIV PMC_PCR_DIV2
#elif (BOARD_MCK_FREQUENCY >> 2) <= SAM_SSC_MAXPERCLK
#  define SSC_FREQUENCY (BOARD_MCK_FREQUENCY >> 2)
#  define SSC_PCR_DIV PMC_PCR_DIV4
#elif (BOARD_MCK_FREQUENCY >> 3) <= SAM_SSC_MAXPERCLK
#  define SSC_FREQUENCY (BOARD_MCK_FREQUENCY >> 3)
#  define SSC_PCR_DIV PMC_PCR_DIV8
#else
#  error Cannot realize SSC input frequency
#endif

/* Clock source definitions */

#define SSC_CLKSRC_NONE   0 /* No clock */
#define SSC_CLKSRC_MCKDIV 1 /* Clock source is MCK divided down */
#define SSC_CLKSRC_RXOUT  2 /* Transmitter clock source is the receiver clock */
#define SSC_CLKSRC_TXOUT  2 /* Receiver clock source is the transmitter clock */
#define SSC_CLKSRC_TKIN   3 /* Transmitter clock source is TK */
#define SSC_CLKSRC_RKIN   3 /* Receiver clock source is RK */

/* Clock output definitions */

#define SSC_CLKOUT_NONE   0 /* No output clock */
#define SSC_CLKOUT_CONT   1 /* Continuous */
#define SSC_CLKOUT_XFER   2 /* Only output clock during transfers */

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.  This is set to
 */

#define DMA_TIMEOUT_MS    (800)
#define DMA_TIMEOUT_TICKS ((DMA_TIMEOUT_MS + (MSEC_PER_TICK-1)) / MSEC_PER_TICK)

/* Debug *******************************************************************/
/* Check if SSC debut is enabled (non-standard.. no support in
 * include/debug.h
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_I2S
#  undef CONFIG_SAMA5_SSC_DMADEBUG
#  undef CONFIG_SAMA5_SSC_REGDEBUG
#endif

#ifndef CONFIG_DEBUG_DMA
#  undef CONFIG_SAMA5_SSC_DMADEBUG
#endif

#ifdef CONFIG_DEBUG_I2S
#  define i2sdbg         dbg
#  define i2slldbg       lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define i2svdbg      dbg
#    define i2sllvdbg    lldbg
#  else
#    define i2svdbg(x...)
#  endif
#else
#  define i2sdbg(x...)
#  define i2slldbg(x...)
#  define i2svdbg(x...)
#  define i2sllvdbg(x...)
#endif

#define DMA_INITIAL      0
#define DMA_AFTER_SETUP  1
#define DMA_AFTER_START  2
#define DMA_CALLBACK     3
#define DMA_TIMEOUT      3
#define DMA_END_TRANSFER 4
#define DMA_NSAMPLES     5

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* I2S buffer container */

struct sam_buffer_s
{
  struct sam_buffer_s *flink;  /* Supports a singly linked list */
  i2s_callback_t callback;     /* Function to call when the transfer completes */
  uint32_t timeout;            /* The timeout value to use with DMA transfers */
  void *arg;                   /* The argument to be returned with the callback */
  void *buffer;                /* I/O buffer */
  size_t nbytes;               /* Number of valid bytes in the buffer */
  int result;                  /* The result of the transfer */
};

/* The state of the one SSC peripheral */

struct sam_ssc_s
{
  struct i2s_dev_s dev;        /* Externally visible I2S interface */
  uint32_t base;               /* SSC controller register base address */
  sem_t exclsem;               /* Assures mutually exclusive acess to SSC */
  uint16_t master:1;           /* True: Master mode transfers */
  uint16_t rx:1;               /* True: RX transfers supported */
  uint16_t tx:1;               /* True: TX transfers supported */
  uint16_t sscno:1;            /* SSC controller number (0 or 1) */
  uint16_t rxclk:2;            /* Receiver clock source. See SSC_CLKSRC_* definitions */
  uint16_t txclk:2;            /* Transmitter clock source. See SSC_CLKSRC_* definitions */
  uint16_t rxout:2;            /* Receiver clock output. See SSC_CLKOUT_* definitions */
  uint16_t txout:2;            /* Transmitter clock output. See SSC_CLKOUT_* definitions */
  uint8_t datalen;             /* Data width (2-32) */
  uint8_t pid;                 /* Peripheral ID */
  uint32_t frequency;          /* Target MCK frequency */

#ifdef SSC_HAVE_RX
  DMA_HANDLE rxdma;            /* SSC RX DMA handle */
  WDOG_ID rxdog;               /* Watchdog that handles RX DMA timeouts */
  sq_queue_t rxpend;           /* A queue of pending RX transfers */
  sq_queue_t rxdone;           /* A queue of completed RX transfers */
  struct sam_buffer_s *rxact;  /* The active RX transfer */
  struct work_s rxwork;        /* Supports worker thread RX operations */
#endif
#ifdef SSC_HAVE_TX
  DMA_HANDLE txdma;            /* SSC TX DMA handle */
  WDOG_ID txdog;               /* Watchdog that handles TX DMA timeouts */
  sq_queue_t txpend;           /* A queue of pending TX transfers */
  sq_queue_t txdone;           /* A queue of completed TX transfers */
  struct sam_buffer_s *txact;  /* The active TX transfer */
  struct work_s txwork;        /* Supports worker thread TX operations */
#endif

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                   /* Buffer wait semaphore */
  struct sam_buffer_s *freelist;  /* A list a free buffer containers */
  struct sam_buffer_s containers[SAMA5_SSC_MAXINFLIGHT];

  /* Debug stuff */

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
   bool     wr;                /* Last was a write */
   uint32_t regaddr;           /* Last address */
   uint32_t regval;            /* Last value */
   int      count;             /* Number of times */
#endif /* CONFIG_SAMA5_SSC_REGDEBUG */

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_RX)
  struct sam_dmaregs_s rxdmaregs[DMA_NSAMPLES];
#endif

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_TX)
  struct sam_dmaregs_s txdmaregs[DMA_NSAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
static bool     ssc_checkreg(struct sam_ssc_s *priv, bool wr, uint32_t value,
                  uint32_t address);
#else
# define        ssc_checkreg(priv,wr,value,address) (false)
#endif

static inline uint32_t ssc_getreg(struct sam_ssc_s *priv, unsigned int offset);
static inline void ssc_putreg(struct sam_ssc_s *priv, uint32_t value,
                  unsigned int offset);
static inline uintptr_t ssc_physregaddr(struct sam_ssc_s *priv,
                  unsigned int offset);

#if defined(CONFIG_DEBUG_I2S) && defined(CONFIG_DEBUG_VERBOSE)
static void     ssc_dumpregs(struct sam_ssc_s *priv, const char *msg);
#else
# define        ssc_dumpregs(priv,msg)
#endif

/* Semaphore helpers */

static void     ssc_exclsemtake(FAR struct sam_ssc_s *priv);
#define         ssc_exclsemgive(priv) sem_post(&priv->exclsem)

static void     ssc_bufsemtake(FAR struct sam_ssc_s *priv);
#define         ssc_bufsemgive(priv) sem_post(&priv->bufsem)

/* Buffer container helpers */

static struct sam_buffer_s *
                ssc_bfallocate(struct sam_ssc_s *priv);
static void     ssc_bffree(struct sam_ssc_s *priv,
                  struct sam_buffer_s *bfcontainer);
static void     ssc_bfinitialize(struct sam_ssc_s *priv);

/* DMA support */

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_RX)
#  define       ssc_rxdma_sample(s,i) sam_dmasample((s)->rxdma, &(s)->rxdmaregs[i])
static void     ssc_rxdma_sampleinit(struct sam_ssc_s *priv);
static void     ssc_rxdma_sampledone(struct sam_ssc_s *priv);

#else
#  define       ssc_rxdma_sample(s,i)
#  define       ssc_rxdma_sampleinit(s)
#  define       ssc_rxdma_sampledone(s)

#endif

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_TX)
#  define ssc_txdma_sample(s,i) sam_dmasample((s)->txdma, &(s)->txdmaregs[i])
static void     ssc_txdma_sampleinit(struct sam_ssc_s *priv);
static void     ssc_txdma_sampledone(struct sam_ssc_s *priv);

#else
#  define       ssc_txdma_sample(s,i)
#  define       ssc_txdma_sampleinit(s)
#  define       ssc_txdma_sampledone(s)

#endif

#ifdef SSC_HAVE_RX
static void     ssc_rxdmatimeout(int argc, uint32_t arg);
static int      ssc_rxdmasetup(FAR struct sam_ssc_s *priv);
static void     ssc_rxworker(FAR void *arg);
static void     ssc_rxschedule(FAR struct sam_ssc_s *priv, int result);
static void     ssc_rxcallback(DMA_HANDLE handle, void *arg, int result);
#endif
#ifdef SSC_HAVE_TX
static void     ssc_txdmatimeout(int argc, uint32_t arg);
static int      ssc_txdmasetup(FAR struct sam_ssc_s *priv);
static void     ssc_txworker(FAR void *arg);
static void     ssc_txschedule(FAR struct sam_ssc_s *priv, int result);
static void     ssc_txcallback(DMA_HANDLE handle, void *arg, int result);
#endif

/* I2S methods */

static uint32_t ssc_frequency(FAR struct i2s_dev_s *dev, uint32_t frequency);
static int      ssc_send(FAR struct i2s_dev_s *dev, FAR const void *buffer,
                  size_t nbytes, i2s_callback_t callback, FAR void *arg,
                  uint32_t timeout);
static int      ssc_receive(FAR struct i2s_dev_s *dev, FAR void *buffer,
                  size_t nbytes, i2s_callback_t callback, FAR void *arg,
                  uint32_t timeout);

/* Initialization */

#ifdef SSC_HAVE_RX
static int      ssc_rx_configure(struct sam_ssc_s *priv);
#endif
#ifdef SSC_HAVE_TX
static int      ssc_tx_configure(struct sam_ssc_s *priv);
#endif
#ifdef CONFIG_SAMA5_SSC0
static void     ssc0_configure(struct sam_ssc_s *priv);
#endif
#ifdef CONFIG_SAMA5_SSC1
static void     ssc1_configure(struct sam_ssc_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* I2S device operations */

static const struct i2s_ops_s g_sscops =
{
  .i2s_frequency = ssc_frequency,
  .i2s_send      = ssc_send,
  .i2s_receive   = ssc_receive,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssc_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
static bool ssc_checkreg(struct sam_ssc_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wr &&     /* Same kind of access? */
      value   == priv->regval &&  /* Same value? */
      address == priv->regaddr)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->count++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->count > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", priv->count);
        }

      /* Save information about the new access */

      priv->wr      = wr;
      priv->regval   = value;
      priv->regaddr = address;
      priv->count      = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: ssc_getreg
 *
 * Description:
 *  Read an SSC register
 *
 ****************************************************************************/

static inline uint32_t ssc_getreg(struct sam_ssc_s *priv,
                                  unsigned int offset)
{
  uint32_t address = priv->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
  if (ssc_checkreg(priv, false, value, address))
    {
      lldbg("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: ssc_putreg
 *
 * Description:
 *  Write a value to an SSC register
 *
 ****************************************************************************/

static inline void ssc_putreg(struct sam_ssc_s *priv, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = priv->base + offset;

#ifdef CONFIG_SAMA5_SSC_REGDEBUG
  if (ssc_checkreg(priv, true, value, address))
    {
      lldbg("%08x<-%08x\n", address, value);
    }
#endif

  putreg32(value, address);
}

/****************************************************************************
 * Name: ssc_physregaddr
 *
 * Description:
 *   Return the physical address of an HSMCI register
 *
 ****************************************************************************/

static inline uintptr_t ssc_physregaddr(struct sam_ssc_s *priv,
                                        unsigned int offset)
{
  return sam_physregaddr(priv->base + offset);
}

/****************************************************************************
 * Name: ssc_dumpregs
 *
 * Description:
 *   Dump the contents of all SSC registers
 *
 * Input Parameters:
 *   priv - The SSC controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_I2S) && defined(CONFIG_DEBUG_VERBOSE)
static void ssc_dumpregs(struct sam_ssc_s *priv, const char *msg)
{
  i2svdbg("SSC%d: %s\n", priv->sscno, msg);
  i2svdbg("   CMR:%08x RCMR:%08x RFMR:%08x TCMR:%08x\n",
          getreg32(priv->base + SAM_SSC_CMR_OFFSET),
          getreg32(priv->base + SAM_SSC_RCMR_OFFSET),
          getreg32(priv->base + SAM_SSC_RFMR_OFFSET),
          getreg32(priv->base + SAM_SSC_TCMR_OFFSET));
  i2svdbg("  TFMR:%08x RC0R:%08x RC1R:%08x   SR:%08x\n",
          getreg32(priv->base + SAM_SSC_TFMR_OFFSET),
          getreg32(priv->base + SAM_SSC_RC0R_OFFSET),
          getreg32(priv->base + SAM_SSC_RC1R_OFFSET),
          getreg32(priv->base + SAM_SSC_SR_OFFSET));
  i2svdbg("   IMR:%08x WPMR:%08x WPSR:%08x\n",
          getreg32(priv->base + SAM_SSC_IMR_OFFSET),
          getreg32(priv->base + SAM_SSC_WPMR_OFFSET),
          getreg32(priv->base + SAM_SSC_WPSR_OFFSET));
}
#endif


/****************************************************************************
 * Name: ssc_exclsemtake
 *
 * Description:
 *   Take the exclusive access semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the SSC peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void ssc_exclsemtake(FAR struct sam_ssc_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal.
   */

  do
    {
      ret = sem_wait(&priv->exclsem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: ssc_exclsemtake
 *
 * Description:
 *   Take the buffer semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the SSC peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void ssc_bufsemtake(FAR struct sam_ssc_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal.
   */

  do
    {
      ret = sem_wait(&priv->bufsem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: ssc_bfallocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive acces to the SSC state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct sam_buffer_s *ssc_bfallocate(struct sam_ssc_s *priv)
{
  struct sam_buffer_s *bfcontainer;
  irqstate_t flags;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  ssc_bufsemtake(priv);

  /* Get the buffer from the head of the free list */

  flags = irqsave();
  bfcontainer = priv->freelist;
  ASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->freelist = bfcontainer->flink;
  irqrestore(flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: ssc_bffree
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - SSC state instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the SSC state structure
 *
 ****************************************************************************/

static void ssc_bffree(struct sam_ssc_s *priv, struct sam_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = irqsave();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  irqrestore(flags);

  /* Wake up any threads waiting for a buffer container */

  ssc_bufsemgive(priv);
}

/****************************************************************************
 * Name: ssc_bfinitialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in SSC intialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void ssc_bfinitialize(struct sam_ssc_s *priv)
{
  int i;

  priv->freelist = NULL;
  sem_init(&priv->bufsem, 0, SAMA5_SSC_MAXINFLIGHT);

  for (i = 0; i < SAMA5_SSC_MAXINFLIGHT; i++)
    {
      ssc_bffree(priv, &priv->containers[i]);
    }
}

/****************************************************************************
 * Name: ssc_rxdma_sampleinit
 *
 * Description:
 *   Initialize sampling of RX DMA registers (if CONFIG_SAMA5_SSC_DMADEBUG)
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_RX)
static void ssc_rxdma_sampleinit(struct sam_ssc_s *priv, bool tx)
{
  /* Put contents of register samples into a known state */

  memset(priv->rxdmaregs, 0xff, DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));

  /* Then get the initial samples */

  sam_dmasample(priv->rxdma, &priv->rxdmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: ssc_txdma_sampleinit
 *
 * Description:
 *   Initialize sampling of TX DMA registers (if CONFIG_SAMA5_SSC_DMADEBUG)
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_TX)
static void ssc_txdma_sampleinit(struct sam_ssc_s *priv)
{
  /* Put contents of register samples into a known state */

  memset(priv->txdmaregs, 0xff, DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));

  /* Then get the initial samples */

  sam_dmasample(priv->txdma, &priv->txdmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: ssc_rxdma_sampledone
 *
 * Description:
 *   Dump sampled RX DMA registers
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_RX)
static void ssc_rxdma_sampledone(struct sam_ssc_s *priv)
{
  /* Sample the final registers */

  sam_dmasample(priv->rxdma, &priv->rxdmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */
  /* Initial register values */

  sam_dmadump(priv->rxdma, &priv->rxdmaregs[DMA_INITIAL],
              "RX: Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(priv->rxdma, &priv->rxdmaregs[DMA_AFTER_SETUP],
              "RX: After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(priv->rxdma, &priv->rxdmaregs[DMA_AFTER_START],
              "RX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timedout, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

  if (priv->result == -ETIMEDOUT)
    {
      sam_dmadump(priv->rxdma, &priv->rxdmaregs[DMA_TIMEOUT],
                  "RX: At DMA timeout");
    }
  else
    {
      sam_dmadump(priv->rxdma, &priv->rxdmaregs[DMA_CALLBACK],
                  "RX: At DMA callback");
    }

  sam_dmadump(priv->rxdma, &priv->rxdmaregs[DMA_END_TRANSFER],
              "RX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: ssc_txdma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SSC_DMADEBUG) && defined(SSC_HAVE_TX)
static void ssc_txdma_sampledone(struct sam_ssc_s *priv)
{
  /* Sample the final registers */

  sam_dmasample(priv->txdma, &priv->txdmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */
  /* Initial register values */

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_INITIAL],
              "TX: Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_AFTER_SETUP],
              "TX: After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_AFTER_START],
              "TX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   */

  if (priv->result == -ETIMEDOUT)
    {
      sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_TIMEOUT],
                  "TX: At DMA timeout");
    }
  else
    {
      sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_CALLBACK],
                  "TX: At DMA callback");
    }

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_END_TRANSFER],
              "TX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: ssc_rxdmatimeout
 *
 * Description:
 *   The RX watchdog timeout without completion of the RX DMA.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static void ssc_rxdmatimeout(int argc, uint32_t arg)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample DMA registers at the time of the timeout */

  ssc_rxdma_sample(priv, DMA_TIMEOUT);

  /* Cancel the DMA */
#warning Missing logic

  /* Then schedule completion of the transfer to occur on the worker thread */

  ssc_rxschedule(priv, -ETIMEDOUT);
}
#endif

/****************************************************************************
 * Name: ssc_rxdmasetup
 *
 * Description:
 *   Setup and initiate the next RX DMA transfer
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static int ssc_rxdmasetup(struct sam_ssc_s *priv)
{
  int ret;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (priv->rxact)
    {
      return OK;
    }

  /* Removing the pending RX transfer at the head of the RX pending queue
   * and save it as the new active transfer.
   */
#warning Missing logic

  /* Start the RX DMA */
#warning Missing logic

  /* Start a watchdog to catch DMA timeouts */

  if (priv->txact->timeout > 0)
    {
      ret = wd_start(priv->txdog, priv->txact->timeout,
                     (wdentry_t)ssc_rxdmatimeout, 1, (uint32_t)priv);
      if (ret < 0)
        {
          int errcode = errno;
          i2slldbg("ERROR: wd_start failed: %d\n", errcode);
          return -errcode;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ssc_rxworker
 *
 * Description:
 *   RX transfer done worker
 *
 * Input Parameters:
 *   arg - the SSC device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static void ssc_rxworker(FAR void *arg)
{
  /* When the transfer was started, the active buffer container was removed
   * from the rxpend queue and saved in as rxact.
   *
   * If the DMA completes successfully or fails due to a timeout, the buffer
   * container in rxact will be moved to the end of the rxdone queue and
   * this worker will be started.
   */

  i2svdbg("txact=%p txdone.head=%p\n", priv->txact, priv->tdone.head);
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: ssc_rxschedule
 *
 * Description:
 *   An RX DMA completion or timeout has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static void ssc_rxschedule(struct sam_ssc_s *priv, int result)
{
  int ret;

  /* Upon entry, the transfer that just complete is the one at head at
   * priv->rxact.  It does not reside in any queue.
   */

  DEBUGASSERT(priv->rxact != NULL);

  /* Report the result of the transfer */

  priv->rxact->result = result;

  /* Add the completed buffer to the tail of the txdone queue */

  sq_addlast((sq_entry_t *)priv->rxact, &priv->rxdone);
  priv->rxact = NULL;

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.
   */

  if (work_available(&priv->rxwork))
    {
      /* Schedule the TX DMA done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->rxwork, ssc_rxworker, priv, 0);
      if (ret != 0)
        {
          i2slldbg("ERROR: Failed to queue RX work: %d\n", ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: ssc_rxcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the SSC RX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SSC_HAVE_RX
static void ssc_rxcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  (void)wd_cancel(priv->rxdog);

  /* Sample DMA registers at the time of the DMA completion */

  ssc_rxdma_sample(priv, DMA_CALLBACK);

  /* Then schedule completion of the transfer to occur on the worker thread */

  ssc_rxschedule(priv, result);
}
#endif

/****************************************************************************
 * Name: ssc_txdmatimeout
 *
 * Description:
 *   The RX watchdog timeout without completion of the RX DMA.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static void ssc_txdmatimeout(int argc, uint32_t arg)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample DMA registers at the time of the timeout */

  ssc_txdma_sample(priv, DMA_TIMEOUT);

  /* Cancel the DMA */
#warning Missing logic

  /* Then schedule completion of the transfer to occur on the worker thread */

  ssc_txschedule(priv, -ETIMEDOUT);
}
#endif

/****************************************************************************
 * Name: ssc_txdmasetup
 *
 * Description:
 *   Setup and initiate the next TX DMA transfer
 *
 * Input Parameters:
 *   priv - SSC state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static int ssc_txdmasetup(struct sam_ssc_s *priv)
{
  int ret;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (priv->txact)
    {
      return OK;
    }

  /* Removing the pending TX transfer at the head of the TX pending queue
   * and save it as the new active transfer.
   */
#warning Missing logic

  /* Start the TX DMA */
#warning Missing logic

  /* Start a watchdog to catch DMA timeouts */

  if (priv->txact->timeout > 0)
    {
      ret = wd_start(priv->txdog, priv->txact->timeout,
                     (wdentry_t)ssc_rxdmatimeout, 1, (uint32_t)priv);
      if (ret < 0)
        {
          int errcode = errno;
          i2slldbg("ERROR: wd_start failed: %d\n", errcode);
          return -errcode;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ssc_txworker
 *
 * Description:
 *   TX transfer done worker
 *
 * Input Parameters:
 *   arg - the SSC device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static void ssc_txworker(FAR void *arg)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer container was removed
   * from the txpend queue and saved in as txact.
   *
   * If the DMA completes successfully or fails due to a timeout, the buffer
   * container in txact will be moved to the end of the txdone queue and
   * this worker will be started.
   */

  i2svdbg("txact=%p txdone.head=%p\n", priv->txact, priv->tdone.head);
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: ssc_txschedule
 *
 * Description:
 *   An TX DMA completion or timeout has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - The TX timeout has been canceled.
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static void ssc_txschedule(struct sam_ssc_s *priv, int result)
{
  int ret;

  /* Upon entry, the transfer that just complete is the one at head at
   * priv->txact.  It does not reside in any queue.
   */

  DEBUGASSERT(priv->txact != NULL);

  /* Report the result of the transfer */

  priv->txact->result = result;

  /* Add the completed buffer to the tail of the txdone queue */

  sq_addlast((sq_entry_t *)priv->txact, &priv->txdone);
  priv->txact = NULL;

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.
   */

  if (work_available(&priv->txwork))
    {
      /* Schedule the TX DMA done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->txwork, ssc_txworker, priv, 0);
      if (ret != 0)
        {
          i2slldbg("ERROR: Failed to queue TX work: %d\n", ret);
        }
    }
}
#endif

/****************************************************************************
 * Name: ssc_txcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the SSC TX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SSC_HAVE_TX
static void ssc_txcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  (void)wd_cancel(priv->txdog);

  /* Sample DMA registers at the time of the DMA completion */

  ssc_txdma_sample(priv, DMA_CALLBACK);

  /* Then schedule completion of the transfer to occur on the worker thread */

  ssc_txschedule(priv, result);
}
#endif

/****************************************************************************
 * Name: ssc_frequency
 *
 * Description:
 *   Set the SSC frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SSC frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t ssc_frequency(struct i2s_dev_s *dev, uint32_t frequency)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
  uint32_t actual;
  uint32_t regval;

  i2svdbg("Frequency=%d\n", frequency);
#warning Missing logic

  i2sdbg("Frequency %d->%d\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: ssc_send
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nbytes - the length of data to send from the buffer in number of bytes.
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int ssc_send(FAR struct i2s_dev_s *dev, FAR const void *buffer,
                    size_t nbytes, i2s_callback_t callback,
                    FAR void *arg, uint32_t timeout)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
  struct sam_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  i2svdbg("buffer=%p nbytes=%d\n", buffer, (int)nbytes);
  DEBUGASSERT(priv);

#ifdef SSC_HAVE_TX
  /* Allocate a buffer container in advance */

  bfcontainer = ssc_bfallocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SSC driver data */

  ssc_exclsemtake(priv);

  /* Has the TX channel been configured? */

  if (!priv->tx)
    {
      i2sdbg("ERROR: SSC%d has no transmitter\n", priv->sscno);
      ret = -EAGAIN;
      goto errout_with_exclsem;
    }

  /* Initialize the buffer container structure */

  bfcontainer->callback = callback; /* Function to call when the transfer completes */
  bfcontainer->timeout  = timeout;  /* The timeout value to use with DMA transfers */
  bfcontainer->arg      = arg;      /* The argument to be returned with the callback */
  bfcontainer->buffer   = (FAR void *)buffer;   /* I/O buffer */
  bfcontainer->nbytes   = nbytes;   /* Number of valid bytes in the buffer */
  bfcontainer->result   = -EBUSY;   /* The result of the transfer */

  /* Add the buffer container to the end of the TX pending queue */

  flags = irqsave();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->txpend);

  /* Then start the next transfer.  If there is already a transfer in progess,
   * then this will do nothing.
   */

  ret = ssc_txdmasetup(priv);
  DEBUGASSERT(ret == OK);
  irqrestore(flags);
  ssc_exclsemgive(priv);
  return OK;

errout_with_exclsem:
  ssc_exclsemgive(priv);
  ssc_bffree(priv, bfcontainer);
  return ret;

#else
  i2sdbg("ERROR: SSC%d has no transmitter\n", priv->sscno);
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: ssc_receive
 *
 * Description:
 *   Receive a block of data from I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   buffer   - A pointer to the buffer in which to recieve data
 *   nbytes   - The length of data that can be received in the buffer in number
 *              of bytes.
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int ssc_receive(FAR struct i2s_dev_s *dev, FAR void *buffer,
                       size_t nbytes, i2s_callback_t callback,
                       FAR void *arg, uint32_t timeout)
{
  struct sam_ssc_s *priv = (struct sam_ssc_s *)dev;
  struct sam_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  i2svdbg("buffer=%p nbytes=%d\n", buffer, (int)nbytes);
  DEBUGASSERT(priv);

#ifdef SSC_HAVE_RX
  /* Allocate a buffer container in advance */

  bfcontainer = ssc_bfallocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SSC driver data */

  ssc_exclsemtake(priv);

  /* Has the RX channel been configured? */

  if (!priv->rx)
    {
      i2sdbg("ERROR: SSC%d has no receiver\n", priv->sscno);
      ret = -EAGAIN;
      goto errout_with_exclsem;
    }

  /* Initialize the buffer container structure */

  bfcontainer->callback = callback; /* Function to call when the transfer completes */
  bfcontainer->timeout  = timeout;  /* The timeout value to use with DMA transfers */
  bfcontainer->arg      = arg;      /* The argument to be returned with the callback */
  bfcontainer->buffer   = buffer;   /* I/O buffer */
  bfcontainer->nbytes   = nbytes;   /* Number of valid bytes in the buffer */
  bfcontainer->result   = -EBUSY;   /* The result of the transfer */

  /* Add the buffer container to the end of the RX pending queue */

  flags = irqsave();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->rxpend);

  /* Then start the next transfer.  If there is already a transfer in progess,
   * then this will do nothing.
   */

  ret = ssc_rxdmasetup(priv);
  DEBUGASSERT(ret == OK);
  irqrestore(flags);
  ssc_exclsemgive(priv);
  return OK;

errout_with_exclsem:
  ssc_exclsemgive(priv);
  ssc_bffree(priv, bfcontainer);
  return ret;

#else
  i2sdbg("ERROR: SSC%d has no receiver\n", priv->sscno);
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: ssc_rx/tx_configure
 *
 * Description:
 *   Configure the SSC receiver and transmitter.
 *
 * Input Parameters:
 *   priv - Fully initialized SSC device structure.
 *
 * Returned Value:
 *   OK is returned on failure.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int ssc_rx_configure(struct sam_ssc_s *priv)
{
#ifdef SSC_HAVE_RX
  uint32_t regval;

  /* RCMR settings */
 /* Configure the receiver input clock */

  regval = 0;
  switch (priv->rxclk)
    {
    case SSC_CLKSRC_RKIN:    /* Receiver clock source is RK */
      regval = SSC_RCMR_CKS_RK;
      break;

    case SSC_CLKSRC_TXOUT:   /* Receiver clock source is the transmitter clock */
      regval = SSC_RCMR_CKS_TK;
      break;

    case SSC_CLKSRC_MCKDIV:  /* Clock source is MCK divided down */
      DEBUGASSERT(priv->frequency != 0);
      regval = SSC_RCMR_CKS_MCK;
      break;

    case SSC_CLKSRC_NONE: /* No clock */
    default:
      i2sdbg("ERROR:  No receiver clock\n");
      return -EINVAL;
    }

  /* Configure the receiver output clock */

  switch (priv->rxout)
    {
    case SSC_CLKOUT_CONT: /* Continuous */
      regval |= SSC_RCMR_CKO_CONT;
      break;

    case SSC_CLKOUT_XFER: /* Only output clock during transfers */
      regval |= SSC_RCMR_CKO_TRANSFER;
      break;

    case SSC_CLKOUT_NONE: /* No output clock */
      regval |= SSC_RCMR_CKO_NONE;
      break;

    default:
      i2sdbg("ERROR: Invalid clock output selection\n");
      return -EINVAL;
    }

  /* REVISIT:  Some of these settings will need to be configurable as well.
   * Currently hardcoded to:
   *
   *   SSC_RCMR_CKI         Receive clock inversion
   *   SSC_RCMR_CKG_CONT    No receive clock gating
   *   SSC_RCMR_START_EDGE  Detection of any edge on RF signal
   *   SSC_RCMR_STOP        Not selected
   *   SSC_RCMR_STTDLY(1)   Receive start delay = 1
   *   SSC_RCMR_PERIOD(0)   Receive period divider = 0
   */

  regval |= (SSC_RCMR_CKI | SSC_RCMR_CKG_CONT | SSC_RCMR_START_EDGE |
             SSC_RCMR_STTDLY(1) | SSC_RCMR_PERIOD(0));
  ssc_putreg(priv, SAM_SSC_RCMR_OFFSET, regval);

  /* RFMR settings. Some of these settings will need to be configurable as well.
   * Currently hardcoded to:
   *
   *  SSC_RFMR_DATLEN(n)    'n' deterimined by configuration
   *  SSC_RFMR_LOOP         Loop mode not selected
   *  SSC_RFMR_MSBF         Most significant bit first
   *  SSC_RFMR_DATNB(n)     Data number 'n' per frame (hard-coded)
   *  SSC_RFMR_FSLEN(0)     Receive frame sync length = 0
   *  SSC_RFMR_FSOS_NONE    RF pin is always in input
   *  SSC_RFMR_FSEDGE_POS   Positive frame sync edge detection
   *  SSC_RFMR_FSLENEXT(0)  FSLEN field extension = 0
   */

  regval = (SSC_RFMR_DATLEN(CONFIG_SAMA5_SSC0_DATALEN - 1) | SSC_RFMR_MSBF |
            SSC_RFMR_DATNB(SSC_DATNB - 1) | SSC_RFMR_FSLEN(0) |
            SSC_RFMR_FSOS_NONE | SSC_RFMR_FSLENEXT(0));
  ssc_putreg(priv, SAM_SSC_RFMR_OFFSET, regval);

#else
  ssc_putreg(priv, SAM_SSC_RCMR_OFFSET, 0);
  ssc_putreg(priv, SAM_SSC_RFMR_OFFSET, 0);

#endif

  /* Disable the receiver */

  ssc_putreg(priv, SAM_SSC_CR_OFFSET, SSC_CR_RXDIS);
  return OK;
}

static int ssc_tx_configure(struct sam_ssc_s *priv)
{
#ifdef SSC_HAVE_TX
  uint32_t regval;

  /* TCMR settings */
  /* Configure the transmitter input clock */

  regval = 0;
  switch (priv->txclk)
    {
    case SSC_CLKSRC_TKIN:    /* Transmitter clock source is TK */
      regval = SSC_TCMR_CKS_TK;
      break;

    case SSC_CLKSRC_RXOUT:   /* Transmitter clock source is the receiver clock */
      regval = SSC_TCMR_CKS_RK;
      break;

    case SSC_CLKSRC_MCKDIV:  /* Clock source is MCK divided down */
      DEBUGASSERT(priv->frequency != 0);
      regval = SSC_TCMR_CKS_MCK;
      break;

    case SSC_CLKSRC_NONE: /* No clock */
    default:
      i2sdbg("ERROR:  No transmitter clock\n");
      return -EINVAL;
    }

  /* Configure the receiver output clock */

  switch (priv->txout)
    {
    case SSC_CLKOUT_CONT: /* Continuous */
      regval |= SSC_TCMR_CKO_CONT;
      break;

    case SSC_CLKOUT_XFER: /* Only output clock during transfers */
      regval |= SSC_TCMR_CKO_TRANSFER;
      break;

    case SSC_CLKOUT_NONE: /* No output clock */
      regval |= SSC_TCMR_CKO_NONE;
      break;

    default:
      i2sdbg("ERROR: Invalid clock output selection\n");
      return -EINVAL;
    }

  /* REVISIT:  Some of these settings will need to be configurable as well.
   * Currently hardcoded to:
   *
   *   SSC_RCMR_CKI           No transmitter clock inversion
   *   SSC_RCMR_CKG_CONT      No transmit clock gating
   *   SSC_TCMR_STTDLY(1)     Receive start delay = 1
   *
   * If master:
   *   SSC_TCMR_START_FALLING Detection of a falling edge on TF signal
   *   SSC_TCMR_PERIOD(n)     'n' depends on the datawidth
   *
   * If slave:
   *   SSC_TCMR_START_EDGE    Detection of any edge on TF signal
   *   SSC_TCMR_PERIOD(0)     Receive period divider = 0
   */

  if (priv->master)
    {
      regval |= (SSC_TCMR_CKG_CONT | SSC_TCMR_START_FALLING | SSC_TCMR_STTDLY(1) |
                 SSC_TCMR_PERIOD(((CONFIG_SAMA5_SSC0_DATALEN * SSC_DATNB) / 2) - 1));
    }
  else
    {
      regval |= (SSC_TCMR_CKG_CONT | SSC_TCMR_START_EDGE | SSC_TCMR_STTDLY(1) |
                 SSC_TCMR_PERIOD(0));
    }

  ssc_putreg(priv, SAM_SSC_TCMR_OFFSET, regval);

  /* RFMR settings. Some of these settings will need to be configurable as well.
   * Currently hardcoded to:
   *
   *  SSC_TFMR_DATLEN(n)    'n' deterimined by configuration
   *  SSC_TFMR_DATDEF        Data default = 0
   *  SSC_TFMR_MSBF          Most significant bit first
   *  SSC_TFMR_DATNB(n)      Data number 'n' per frame (hard-coded)
   *  SSC_TFMR_FSDEN         Frame sync data is not enabled
   *  SSC_TFMR_FSLENEXT(0)   FSLEN field extension = 0
   *
   * If master:
   *  SSC_TFMR_FSLEN(n)      Receive frame sync length depends on data width
   *  SSC_TFMR_FSOS_NEGATIVE Negative pulse TF output
   *
   * If slave:
   *  SSC_TFMR_FSLEN(0)      Receive frame sync length = 0
   *  SSC_TFMR_FSOS_NONE     TF is an output
   */

  if (priv->master)
    {
      regval = (SSC_TFMR_DATLEN(CONFIG_SAMA5_SSC0_DATALEN - 1) |
                SSC_TFMR_MSBF | SSC_TFMR_DATNB(SSC_DATNB - 1) |
                SSC_TFMR_FSLEN(CONFIG_SAMA5_SSC0_DATALEN - 1) |
                SSC_TFMR_FSOS_NEGATIVE | SSC_TFMR_FSLENEXT(0));
    }
  else
    {
      regval = (SSC_TFMR_DATLEN(CONFIG_SAMA5_SSC0_DATALEN - 1) |
                SSC_TFMR_MSBF | SSC_TFMR_DATNB(SSC_DATNB - 1) |
                SSC_TFMR_FSLEN(0) | SSC_TFMR_FSOS_NONE |
                SSC_TFMR_FSLENEXT(0));
    }

  ssc_putreg(priv, SAM_SSC_TFMR_OFFSET, regval);

#else
  ssc_putreg(priv, SAM_SSC_TCMR_OFFSET, 0);
  ssc_putreg(priv, SAM_SSC_TFMR_OFFSET, 0);

#endif

  /* Disable the transmitter */

  ssc_putreg(priv, SAM_SSC_CR_OFFSET, SSC_CR_TXDIS);
  return OK;
}

/****************************************************************************
 * Name: ssc0/1_configure
 *
 * Description:
 *   Configure SSC0 and/or SSC1
 *
 * Input Parameters:
 *   priv - Partially initialized I2C device structure.  These functions
 *          will compolete the SSC specific portions of the initialization
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SSC0
static void ssc0_configure(struct sam_ssc_s *priv)
{
  /* Configure multiplexed pins as connected on the board.  Chip
   * select pins must be selected by board-specific logic.
   */

#ifdef CONFIG_SAMA5_SSC0_RX
  priv->rx = true;

  /* Configure the receiver data (RD) and receiver frame synchro (RF) pins */

  sam_configpio(PIO_SSC0_RD);
  sam_configpio(PIO_SSC0_RF);

#if defined(CONFIG_SAMA5_SSC0_RX_RKINPUT)
  /* Configure the RK pin only if we are using an external clock to drive
   * the receiver clock.
   *
   * REVISIT:  The SSC is also capable of generated the receiver clock
   * output on the RK pin.
   */

  sam_configpio(PIO_SSC0_RK); /* External clock received on the RK I/O pad */
  priv->rxclk = SSC_CLKSRC_RKIN;

#elif defined(CONFIG_SAMA5_SSC0_RX_TXCLK)
  priv->rxclk = SSC_CLKSRC_TXOUT;

#elif defined(CONFIG_SAMA5_SSC0_RX_MCKDIV)
  priv->rxclk = SSC_CLKSRC_MCKDIV;

#else
  priv->rxclk = SSC_CLKSRC_NONE;

#endif

  /* Remember the configured RX clock output */

#if defined(CONFIG_SAMA5_SSC0_RX_RKOUTPUT_CONT)
  priv->rxout = SSC_CLKOUT_CONT; /* Continuous */
#elif defined(CONFIG_SAMA5_SSC0_RX_RKOUTPUT_XFR)
  priv->rxout = SSC_CLKOUT_XFER; /* Only output clock during transfers */
#else /* if defined(CONFIG_SAMA5_SSC0_RX_RKOUTPUT_NONE) */
  priv->rxout = SSC_CLKOUT_NONE; /* No output clock */
#endif

#else
  priv->rx    = false;
  priv->rxclk = SSC_CLKSRC_NONE; /* No input clock */
  priv->rxout = SSC_CLKOUT_NONE; /* No output clock */

#endif /* CONFIG_SAMA5_SSC0_RX */

#ifdef CONFIG_SAMA5_SSC0_TX
  priv->tx = true;

  /* Configure the transmitter data (TD) and transmitter frame synchro (TF)
   * pins
   */

  sam_configpio(PIO_SSC0_TD);
  sam_configpio(PIO_SSC0_TF);

#if defined(CONFIG_SAMA5_SSC0_TX_TKINPUT)
  /* Configure the TK pin only if we are using an external clock to drive
   * the transmitter clock.
   *
   * REVISIT:  The SSC is also capable of generated the transmitter clock
   * output on the TK pin.
   */

  sam_configpio(PIO_SSC0_TK); /* External clock received on the TK I/O pad */
  priv->txclk = SSC_CLKSRC_TKIN;

#elif defined(CONFIG_SAMA5_SSC0_TX_RXCLK)
  priv->txclk = SSC_CLKSRC_RXOUT;

#elif defined(CONFIG_SAMA5_SSC0_TX_MCKDIV)
  priv->txclk = SSC_CLKSRC_MCKDIV;

#else
  priv->txclk = SSC_CLKSRC_NONE;

#endif

  /* Remember the configured TX clock output */

#if defined(CONFIG_SAMA5_SSC0_TX_TKOUTPUT_CONT)
  priv->txout = SSC_CLKOUT_CONT; /* Continuous */
#elif defined(CONFIG_SAMA5_SSC0_TX_TKOUTPUT_XFR)
  priv->txout = SSC_CLKOUT_XFER; /* Only output clock during transfers */
#else /* if defined(CONFIG_SAMA5_SSC0_TX_TKOUTPUT_NONE) */
  priv->txout = SSC_CLKOUT_NONE; /* No output clock */
#endif

#else
  priv->tx    = false;
  priv->txclk = SSC_CLKSRC_NONE; /* No input clock */
  priv->txout = SSC_CLKOUT_NONE; /* No output clock */

#endif /* CONFIG_SAMA5_SSC0_TX */

  /* Does the receiver or transmitter need to have the MCK divider set up? */

#if (defined(CONFIG_SAMA5_SSC0_RX) && defined(CONFIG_SAMA5_SSC0_RX_MCKDIV)) || \
    (defined(CONFIG_SAMA5_SSC0_TX) && defined(CONFIG_SAMA5_SSC0_TX_MCKDIV))

  priv->frequency = CONFIG_SAMA5_SSC0_MCKDIV_FREQUENCY;

#else
  priv->frequency = 0;

#endif

  /* Configure driver state specific to this SSC peripheral */

  priv->base    = SAM_SSC0_VBASE;
#ifdef CONFIG_SAMA5_SSC0_MASTER
  priv->master  = true;
#else
  priv->master  = false;
#endif
  priv->datalen = CONFIG_SAMA5_SSC0_DATALEN;
  priv->pid     = SAM_PID_SSC0;
}
#endif

#ifdef CONFIG_SAMA5_SSC1
static void ssc1_configure(struct sam_ssc_s *priv)
{
  /* Configure multiplexed pins as connected on the board.  Chip
   * select pins must be selected by board-specific logic.
   */

#ifdef CONFIG_SAMA5_SSC1_RX
  priv->rx = true;

  /* Configure the receiver data (RD) and receiver frame synchro (RF) pins */

  sam_configpio(PIO_SSC1_RD);
  sam_configpio(PIO_SSC1_RF);

#ifdef CONFIG_SAMA5_SSC1_RX_RKINPUT
  /* Configure the RK pin only if we are using an external clock to drive
   * the receiver clock.
   *
   * REVISIT:  The SSC is also capable of generated the receiver clock
   * output on the RK pin.
   */

  sam_configpio(PIO_SSC1_RK); /* External clock received on the RK I/O pad */
  priv->rxclk = SSC_CLKSRC_RKIN;

#elif defined(CONFIG_SAMA5_SSC1_RX_TXCLK)
  priv->rxclk = SSC_CLKSRC_TXOUT;

#elif defined(CONFIG_SAMA5_SSC1_RX_MCKDIV)
  priv->rxclk = SSC_CLKSRC_MCKDIV;

#else
  priv->rxclk = SSC_CLKSRC_NONE;

#endif

  /* Remember the configured RX clock output */

#if defined(CONFIG_SAMA5_SSC1_RX_RKOUTPUT_CONT)
  priv->rxout = SSC_CLKOUT_CONT; /* Continuous */
#elif defined(CONFIG_SAMA5_SSC1_RX_RKOUTPUT_XFR)
  priv->rxout = SSC_CLKOUT_XFER; /* Only output clock during transfers */
#else /* if defined(CONFIG_SAMA5_SSC1_RX_RKOUTPUT_NONE) */
  priv->rxout = SSC_CLKOUT_NONE; /* No output clock */
#endif

#else
  priv->rx    = false;
  priv->rxclk = SSC_CLKSRC_NONE; /* No input clock */
  priv->rxout = SSC_CLKOUT_NONE; /* No output clock */

#endif /* CONFIG_SAMA5_SSC1_RX */

#ifdef CONFIG_SAMA5_SSC1_TX
  priv->tx = true;

  /* Configure the transmitter data (TD) and transmitter frame synchro (TF)
   * pins
   */

  sam_configpio(PIO_SSC1_TD);
  sam_configpio(PIO_SSC1_TF);

#if defined(CONFIG_SAMA5_SSC1_TX_TKINPUT)
  /* Configure the TK pin only if we are using an external clock to drive
   * the transmitter clock.
   *
   * REVISIT:  The SSC is also capable of generated the transmitter clock
   * output on the TK pin.
   */

  sam_configpio(PIO_SSC1_TK); /* External clock received on the TK I/O pad */
  priv->txclk = SSC_CLKSRC_TKIN;

#elif defined(CONFIG_SAMA5_SSC1_TX_RXCLK)
  priv->txclk = SSC_CLKSRC_RXOUT;

#elif defined(CONFIG_SAMA5_SSC1_TX_MCKDIV)
  priv->txclk = SSC_CLKSRC_MCKDIV;

#else
  priv->txclk = SSC_CLKSRC_NONE;

#endif

  /* Remember the configured TX clock output */

#if defined(CONFIG_SAMA5_SSC1_TX_TKOUTPUT_CONT)
  priv->txout = SSC_CLKOUT_CONT; /* Continuous */
#elif defined(CONFIG_SAMA5_SSC1_TX_TKOUTPUT_XFR)
  priv->txout = SSC_CLKOUT_XFER;/* Only output clock during transfers */
#else /* if defined(CONFIG_SAMA5_SSC1_TX_TKOUTPUT_NONE) */
  priv->txout = SSC_CLKOUT_NONE; /* No output clock */
#endif

#else
  priv->tx    = false;
  priv->txclk = SSC_CLKSRC_NONE; /* No input clock */
  priv->txout = SSC_CLKOUT_NONE; /* No output clock */

#endif /* CONFIG_SAMA5_SSC1_TX */

  /* Does the receiver or transmitter need to have the MCK divider set up? */

#if (defined(CONFIG_SAMA5_SSC1_RX) && defined(CONFIG_SAMA5_SSC1_RX_MCKDIV)) || \
    (defined(CONFIG_SAMA5_SSC1_TX) && defined(CONFIG_SAMA5_SSC1_TX_MCKDIV))

  priv->frequency = CONFIG_SAMA5_SSC1_MCKDIV_FREQUENCY;

#else
  priv->frequency = 0;

#endif

  /* Configure driver state specific to this SSC peripheral */

  priv->base    = SAM_SSC1_VBASE;
#ifdef CONFIG_SAMA5_SSC1_MASTER
  priv->master  = true;
#else
  priv->master  = false;
#endif
  priv->datalen = CONFIG_SAMA5_SSC1_DATALEN;
  priv->pid     = SAM_PID_SSC1;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_ssc_initialize
 *
 * Description:
 *   Initialize the selected SSC port
 *
 * Input Parameter:
 *   port - I2S "port" number (identifying the "logical" SSC port)
 *
 * Returned Value:
 *   Valid SSC device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *sam_ssc_initialize(int port)
{
  struct sam_ssc_s *priv;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  /* The support SAM parts have only a single SSC port */

  i2svdbg("port: %d\n", port);

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */

  priv = (struct sam_ssc_s *)zalloc(sizeof(struct sam_ssc_s));
  if (!priv)
    {
      i2sdbg("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by zalloc().
   */

  /* Initialize the common parts for the SSC device structure  */

  sem_init(&priv->exclsem, 0, 1);
  priv->dev.ops = &g_sscops;
  priv->sscno   = port;

  /* Initialize buffering */

  ssc_bfinitialize(priv);

  flags = irqsave();
#ifdef CONFIG_SAMA5_SSC0
  if (port == 0)
    {
      ssc0_configure(priv);
    }
  else
#endif /* CONFIG_SAMA5_SSC0 */
#ifdef CONFIG_SAMA5_SSC1
  if (port == 1)
    {
      ssc1_configure(priv);
    }
  else
#endif /* CONFIG_SAMA5_SSC1 */
    {
      i2sdbg("ERROR:  Unsupported I2S port: %d\n", port);
      goto errout_with_alloc;
    }

  /* Allocate DMA channels.  These allocations exploit that fact that
   * SSC0 is managed by DMAC0 and SSC1 is managed by DMAC1.  Hence,
   * the SSC number (port) is the same as the DMAC number.
   */

#ifdef SSC_HAVE_RX
  if (priv->rx)
    {
      /* Allocate an RX DMA channel */

      priv->rxdma = sam_dmachannel(port, 0);
      if (!priv->rxdma)
        {
          i2sdbg("ERROR: Failed to allocate the RX DMA channel\n");
          goto errout_with_alloc;
        }

      /* Create a watchdog time to catch RX DMA timeouts */

      priv->rxdog = wd_create();
      if (!priv->rxdog)
        {
          i2sdbg("ERROR: Failed to create the RX DMA watchdog\n");
          goto errout_with_rxdma;
        }
    }
#endif

#ifdef SSC_HAVE_TX
  if (priv->tx)
    {
      /* Allocate a TX DMA channel */

      priv->txdma = sam_dmachannel(port, 0);
      if (!priv->txdma)
        {
          i2sdbg("ERROR: Failed to allocate the TX DMA channel\n");
          goto errout_with_rxdog;
        }

      /* Create a watchdog time to catch TX DMA timeouts */

      priv->txdog = wd_create();
      if (!priv->txdog)
        {
          i2sdbg("ERROR: Failed to create the TX DMA watchdog\n");
          goto errout_with_txdma;
        }
    }
#endif

  /* Initialize I2S hardware */
  /* Set the maximum SSC peripheral clock frequency */

  regval = PMC_PCR_PID(priv->pid) | PMC_PCR_CMD | SSC_PCR_DIV | PMC_PCR_EN;
  putreg32(regval, SAM_PMC_PCR);

  /* Reset, disable receiver & transmitter */

  ssc_putreg(priv, SAM_SSC_CR_OFFSET, SSC_CR_RXDIS | SSC_CR_TXDIS | SSC_CR_SWRST);

  /* Configure MCK/2 divider */

  if (priv->frequency > 0)
    {
      regval = SSC_FREQUENCY / (2 * priv->frequency);
    }
  else
    {
      regval = 0;
    }

  ssc_putreg(priv, SAM_SSC_CMR_OFFSET, regval);

  /* Enable peripheral clocking */

  sam_enableperiph1(priv->pid);

  /* Configure the receiver */

  ret = ssc_rx_configure(priv);
  if (ret < 0)
    {
      i2sdbg("ERROR: Failed to configure the receiver: %d\n", ret);
      goto errout_with_clocking;
    }

  /* Configure the transmitter */

  ret = ssc_tx_configure(priv);
  if (ret < 0)
    {
      i2sdbg("ERROR: Failed to configure the transmitter: %d\n", ret);
      goto errout_with_clocking;
    }

  /* Configure DMA */
#warning "Missing logic"
  irqrestore(flags);
  ssc_dumpregs(priv, "After initialization");

  /* Success exit */

  return &priv->dev;

  /* Failure exits */

errout_with_clocking:
  sam_disableperiph1(priv->pid);

#ifdef SSC_HAVE_TX
  if (priv->txdog)
    {
       wd_delete(priv->txdog);
    }

errout_with_txdma:
  if (priv->txdma)
    {
      sam_dmafree(priv->txdma);
    }
#endif

errout_with_rxdog:
#ifdef SSC_HAVE_RX
  if (priv->rxdog)
    {
       wd_delete(priv->rxdog);
    }
#endif

#ifdef SSC_HAVE_RX
errout_with_rxdma:
  if (priv->rxdma)
    {
      sam_dmafree(priv->rxdma);
    }
#endif

errout_with_alloc:
  sem_destroy(&priv->exclsem);
  kfree(priv);
  return NULL;
}

#endif /* CONFIG_SAMA5_SSC0 || CONFIG_SAMA5_SSC1 */
