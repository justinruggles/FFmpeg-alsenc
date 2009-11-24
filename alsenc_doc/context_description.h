/** Detailed declaration and description of the encoder's context structure
 */

typedef struct {
    AVCodecContext *avctx;
    ALSSpecificConfig sconf;     ///< Maybe the very same struct like in the decoder
                                 ///< to be filled with information from
                                 ///<  - frame partitioning
                                 ///<  - block partitioning
                                 ///<  - channel sorting
                                 ///<  - preprocessing (0-lsb's -> shift_lsbs)
    GetBitContext gb;
} ALSEncContext;

