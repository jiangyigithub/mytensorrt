//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/image
//          Synopsis: 
//  Target system(s): 
//       Compiler(s): VS7.1
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes: 
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: zvh2hi
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_image_algorithm.inl $
///     $Revision: 1.25 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/11/26 16:55:09MEZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello  $
///     $State: In_Development $
///
/// @par Review Information:
/// - Reviewed version: 
/// - Type (use 'X' to mark):
///     - [ ] Formal Review
///     - [ ] Walkthrough
///     - [ ] Inspection
/// - State including date (DD.MM.YYYY)
///     - [--.--.----] Preparation
///     - [--.--.----] Review audit
///     - [--.--.----] Integration of findings
///     - [--.--.----] Test
///     - [--.--.----] Verification of integration of findings
///     - [--.--.----] Review release
/// - Responsible: 
/// - Review-Document: 
//=============================================================================

#include "vfc/core/vfc_static_assert.hpp"
#include "vfc/core/vfc_assert.hpp"   

#include "vfc/image/vfc_image_traits.hpp"   // used for TImageRawIteratorTraits

template <  class ImageType, 
            class FunctorType>
void    vfc::for_each_pixel (   ImageType& f_img, 
                                FunctorType& f_func)
{
    typedef typename TImageRawIteratorTraits<ImageType>::raw_iterator raw_iterator_type;

    raw_iterator_type rowiter   = f_img.begin_raw();
    raw_iterator_type rowend    = f_img.end_raw();

    raw_iterator_type coliter, colendunaligned, colend;

    // calc unaligned span length
    int32_t    unaligned = (f_img.getWidth()&0x0F);

    for (;rowiter != rowend; rowiter += f_img.getStride())
    {
        coliter            = rowiter;
        colendunaligned = rowiter + unaligned;
        colend            = rowiter + f_img.getWidth();

        // single step for unaligned pixels
        for(;   coliter != colendunaligned; 
                ++coliter)
        {
            f_func(coliter);
        }

        // pixels are 16 aligned now
        for(;   coliter != colend; 
                coliter += 16)
        {
            f_func(coliter);    f_func(coliter+1);  f_func(coliter+2);  f_func(coliter+3);
            f_func(coliter+4);  f_func(coliter+5);  f_func(coliter+6);  f_func(coliter+7);
            f_func(coliter+8);  f_func(coliter+9);  f_func(coliter+10); f_func(coliter+11);
            f_func(coliter+12); f_func(coliter+13); f_func(coliter+14); f_func(coliter+15);
        }
    }

}

template <  class Image1Type, class Image2Type, 
            class FunctorType>    
void    vfc::for_each_pixel    (    Image1Type& f_img1, Image2Type& f_img2, 
                                    FunctorType& f_func)
{
    VFC_REQUIRE((f_img1.getWidth() == f_img2.getWidth())
        && (f_img1.getHeight()== f_img2.getHeight()));

    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1     = f_img1.end_raw();

    first_raw_iterator_type coliter1, colendunaligned1, colend1;

    second_raw_iterator_type rowiter2    = f_img2.begin_raw();
    second_raw_iterator_type coliter2;

    // calc unaligned span length
    int32_t    unaligned = (f_img1.getWidth()&0x0F);

    for (;  rowiter1 != rowend1; 
            rowiter1 += f_img1.getStride(), rowiter2 += f_img2.getStride())
    {
        coliter1        = rowiter1;
        colendunaligned1= rowiter1 + unaligned;
        colend1         = rowiter1 + f_img1.getWidth();

        coliter2        = rowiter2;

        // single step for unaligned pixels
        for(;   coliter1 != colendunaligned1; 
                ++coliter1, ++coliter2)
        {
            f_func(coliter1,coliter2);
        }

        // pixels are now 16 aligned
        for(;   coliter1 != colend1; 
                coliter1 += 16, coliter2 += 16)
        {
            f_func( coliter1,    coliter2);    f_func( coliter1+1,  coliter2+1);    
            f_func( coliter1+2,  coliter2+2);  f_func( coliter1+3,  coliter2+3);
            f_func( coliter1+4,  coliter2+4);  f_func( coliter1+5,  coliter2+5);    
            f_func( coliter1+6,  coliter2+6);  f_func( coliter1+7,  coliter2+7);
            f_func( coliter1+8,  coliter2+8);  f_func( coliter1+9,  coliter2+9);    
            f_func( coliter1+10, coliter2+10); f_func( coliter1+11, coliter2+11);
            f_func( coliter1+12, coliter2+12); f_func( coliter1+13, coliter2+13);    
            f_func( coliter1+14, coliter2+14); f_func( coliter1+15, coliter2+15);
        }
    }

}

template <  class Image1Type, class Image2Type, class Image3Type, 
            class FunctorType>
void    vfc::for_each_pixel (   Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                FunctorType& f_func)
{
    VFC_REQUIRE(    (f_img1.getWidth()  == f_img2.getWidth())
        &&  (f_img1.getWidth()  == f_img3.getWidth())
        &&  (f_img1.getHeight() == f_img2.getHeight())
        &&  (f_img1.getHeight() == f_img3.getHeight()));
    
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image3Type>::raw_iterator third_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1     = f_img1.end_raw();

    first_raw_iterator_type coliter1, colendunaligned1, colend1;

    second_raw_iterator_type rowiter2   = f_img2.begin_raw();
    second_raw_iterator_type coliter2;

    third_raw_iterator_type rowiter3    = f_img3.begin_raw();
    third_raw_iterator_type coliter3;

    // calc unaligned span length
    int32_t    unaligned = (f_img1.getWidth()&0x0F);

    for (;  rowiter1 != rowend1; 
            rowiter1 += f_img1.getStride(), rowiter2 += f_img2.getStride(), 
            rowiter3 += f_img3.getStride())
    {
        coliter1        = rowiter1;
        colendunaligned1= rowiter1 + unaligned;
        colend1         = rowiter1 + f_img1.getWidth();

        coliter2        = rowiter2;
        coliter3        = rowiter3;

        // single step for unaligned pixels
        for(;   coliter1 != colendunaligned1; 
                ++coliter1, ++coliter2, 
                ++coliter3)
        {
            f_func(coliter1, coliter2, coliter3);
        }

        // pixels are now 16 aligned ...
        for(;   coliter1 != colend1; 
                coliter1 += 16, coliter2 += 16, 
                coliter3 += 16)
        {
            f_func( coliter1,    coliter2,    coliter3);    f_func(coliter1+1,  coliter2+1,  coliter3+1);    
            f_func( coliter1+2,  coliter2+2,  coliter3+2);  f_func(coliter1+3,  coliter2+3,  coliter3+3);
            f_func( coliter1+4,  coliter2+4,  coliter3+4);  f_func(coliter1+5,  coliter2+5,  coliter3+5);    
            f_func( coliter1+6,  coliter2+6,  coliter3+6);  f_func(coliter1+7,  coliter2+7,  coliter3+7);
            f_func( coliter1+8,  coliter2+8,  coliter3+8);  f_func(coliter1+9,  coliter2+9,  coliter3+9);    
            f_func( coliter1+10, coliter2+10, coliter3+10); f_func(coliter1+11, coliter2+11, coliter3+11);
            f_func( coliter1+12, coliter2+12, coliter3+12); f_func(coliter1+13, coliter2+13, coliter3+13);    
            f_func( coliter1+14, coliter2+14, coliter3+14); f_func(coliter1+15, coliter2+15, coliter3+15);
        }
    }

}

template <  class Image1Type, class Image2Type, class Image3Type, 
            class Image4Type, 
            class FunctorType>
void    vfc::for_each_pixel (   Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                Image4Type& f_img4, 
                                FunctorType& f_func)
{
    VFC_REQUIRE((f_img1.getWidth()  == f_img2.getWidth())
        &&  (f_img1.getWidth()  == f_img3.getWidth())
        &&  (f_img1.getWidth()  == f_img4.getWidth())
        &&  (f_img1.getHeight() == f_img2.getHeight())
        &&  (f_img1.getHeight() == f_img3.getHeight())
        &&  (f_img1.getHeight() == f_img4.getHeight()));
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image3Type>::raw_iterator third_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image4Type>::raw_iterator fourth_raw_iterator_type;
    
    first_raw_iterator_type rowiter1= f_img1.begin_raw();
    first_raw_iterator_type rowend1 = f_img1.end_raw();

    first_raw_iterator_type coliter1, colendunaligned1, colend1;

    second_raw_iterator_type rowiter2   = f_img2.begin_raw();
    second_raw_iterator_type coliter2;

    third_raw_iterator_type rowiter3    = f_img3.begin_raw();
    third_raw_iterator_type coliter3;

    fourth_raw_iterator_type rowiter4   = f_img4.begin_raw();
    fourth_raw_iterator_type coliter4;

    // calc unaligned span length
    int32_t    unaligned = (f_img1.getWidth()&0x0F);

    for (;  rowiter1 != rowend1; 
            rowiter1 += f_img1.getStride(), rowiter2 += f_img2.getStride(), 
            rowiter3 += f_img3.getStride(), rowiter4 += f_img4.getStride())
    {
        coliter1        = rowiter1;
        colendunaligned1= rowiter1 + unaligned;
        colend1         = rowiter1 + f_img1.getWidth();

        coliter2        = rowiter2;
        coliter3        = rowiter3;
        coliter4        = rowiter4;

        for(;    coliter1 != colendunaligned1; 
                ++coliter1, ++coliter2, 
                ++coliter3, ++coliter4)
        {
            f_func(coliter1, coliter2, coliter3, coliter4);
        }

        for(;   coliter1 != colend1; 
                coliter1 += 16, coliter2 += 16, 
                coliter3 += 16, coliter4 += 16)
        {
            f_func( coliter1,    coliter2,    coliter3,    coliter4);        
            f_func( coliter1+1,  coliter2+1,  coliter3+1,  coliter4+1);    
            f_func( coliter1+2,  coliter2+2,  coliter3+2,  coliter4+2);    
            f_func( coliter1+3,  coliter2+3,  coliter3+3,  coliter4+3);
            f_func( coliter1+4,  coliter2+4,  coliter3+4,  coliter4+4);    
            f_func( coliter1+5,  coliter2+5,  coliter3+5,  coliter4+5);    
            f_func( coliter1+6,  coliter2+6,  coliter3+6,  coliter4+6);    
            f_func( coliter1+7,  coliter2+7,  coliter3+7,  coliter4+7);
            f_func( coliter1+8,  coliter2+8,  coliter3+8,  coliter4+8);    
            f_func( coliter1+9,  coliter2+9,  coliter3+9,  coliter4+9);    
            f_func( coliter1+10, coliter2+10, coliter3+10, coliter4+10);    
            f_func( coliter1+11, coliter2+11, coliter3+11, coliter4+11);
            f_func( coliter1+12, coliter2+12, coliter3+12, coliter4+12);    
            f_func( coliter1+13, coliter2+13, coliter3+13, coliter4+13);    
            f_func( coliter1+14, coliter2+14, coliter3+14, coliter4+14);    
            f_func( coliter1+15, coliter2+15, coliter3+15, coliter4+15);
        }
    }
    
}

template <  class Image1Type, class Image2Type, class Image3Type, 
            class Image4Type, class Image5Type, 
            class FunctorType>
void    vfc::for_each_pixel    (    Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                    Image4Type& f_img4, Image5Type& f_img5, 
                                    FunctorType& f_func)
{
    // soft stop
    VFC_REQUIRE((f_img1.getWidth()  == f_img2.getWidth())
            &&  (f_img1.getWidth()  == f_img3.getWidth())
            &&  (f_img1.getWidth()  == f_img4.getWidth())
            &&  (f_img1.getWidth()  == f_img5.getWidth())
            &&  (f_img1.getHeight() == f_img2.getHeight())
            &&  (f_img1.getHeight() == f_img3.getHeight())
            &&  (f_img1.getHeight() == f_img4.getHeight())
            &&  (f_img1.getHeight() == f_img5.getHeight()));

    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image3Type>::raw_iterator third_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image4Type>::raw_iterator fourth_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image5Type>::raw_iterator fifth_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1     = f_img1.end_raw();

    first_raw_iterator_type coliter1, colendunaligned1, colend1;

    second_raw_iterator_type rowiter2   = f_img2.begin_raw();
    second_raw_iterator_type coliter2;

    third_raw_iterator_type rowiter3    = f_img3.begin_raw();
    third_raw_iterator_type coliter3;

    fourth_raw_iterator_type rowiter4   = f_img4.begin_raw();
    fourth_raw_iterator_type coliter4;

    fifth_raw_iterator_type rowiter5    = f_img5.begin_raw();
    fifth_raw_iterator_type coliter5;

    // calc unaligned span length
    int32_t    unaligned = (f_img1.getWidth()&0x0F);

    for (;      rowiter1 != rowend1; 
                rowiter1 += f_img1.getStride(), rowiter2 += f_img2.getStride(), 
                rowiter3 += f_img3.getStride(), rowiter4 += f_img4.getStride(), 
                rowiter5 += f_img5.getStride())
    {
        coliter1        = rowiter1;
        colendunaligned1= rowiter1 + unaligned;
        colend1         = rowiter1 + f_img1.getWidth();

        coliter2        = rowiter2;
        coliter3        = rowiter3;
        coliter4        = rowiter4;
        coliter5        = rowiter5;

        for(;   coliter1 != colendunaligned1; 
                ++coliter1, ++coliter2, 
                ++coliter3, ++coliter4,
                ++coliter5)
        {
            f_func(coliter1, coliter2, coliter3, coliter4, coliter5);
        }

        for(;   coliter1 != colend1; 
                coliter1 += 16, coliter2 += 16, 
                coliter3 += 16, coliter4 += 16,
                coliter5 += 16)
        {
            f_func(coliter1,    coliter2,    coliter3,    coliter4,     coliter5    );        
            f_func(coliter1+1,  coliter2+1,  coliter3+1,  coliter4+1,   coliter5+1  );    
            f_func(coliter1+2,  coliter2+2,  coliter3+2,  coliter4+2,   coliter5+2  );    
            f_func(coliter1+3,  coliter2+3,  coliter3+3,  coliter4+3,   coliter5+3  );
            f_func(coliter1+4,  coliter2+4,  coliter3+4,  coliter4+4,   coliter5+4  );    
            f_func(coliter1+5,  coliter2+5,  coliter3+5,  coliter4+5,   coliter5+5  );    
            f_func(coliter1+6,  coliter2+6,  coliter3+6,  coliter4+6,   coliter5+6  );    
            f_func(coliter1+7,  coliter2+7,  coliter3+7,  coliter4+7,   coliter5+7  );
            f_func(coliter1+8,  coliter2+8,  coliter3+8,  coliter4+8,   coliter5+8  );    
            f_func(coliter1+9,  coliter2+9,  coliter3+9,  coliter4+9,   coliter5+9  );    
            f_func(coliter1+10, coliter2+10, coliter3+10, coliter4+10,  coliter5+10 );    
            f_func(coliter1+11, coliter2+11, coliter3+11, coliter4+11,  coliter5+11 );
            f_func(coliter1+12, coliter2+12, coliter3+12, coliter4+12,  coliter5+12 );    
            f_func(coliter1+13, coliter2+13, coliter3+13, coliter4+13,  coliter5+13 );    
            f_func(coliter1+14, coliter2+14, coliter3+14, coliter4+14,  coliter5+14 );    
            f_func(coliter1+15, coliter2+15, coliter3+15, coliter4+15,  coliter5+15 );
        }
    }

}

template <  class Image1Type, class Image2Type, class Image3Type, 
            class Image4Type, class Image5Type, class Image6Type, 
            class FunctorType>
void    vfc::for_each_pixel    (    Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                    Image4Type& f_img4, Image5Type& f_img5, Image6Type& f_img6, 
                                    FunctorType& f_func)
{
    // soft stop
    VFC_REQUIRE((f_img1.getWidth()  == f_img2.getWidth())
            &&  (f_img1.getWidth()  == f_img3.getWidth())
            &&  (f_img1.getWidth()  == f_img4.getWidth())
            &&  (f_img1.getWidth()  == f_img5.getWidth())
            &&  (f_img1.getWidth()  == f_img6.getWidth())
            &&  (f_img1.getHeight() == f_img2.getHeight())
            &&  (f_img1.getHeight() == f_img3.getHeight())
            &&  (f_img1.getHeight() == f_img4.getHeight())
            &&  (f_img1.getHeight() == f_img5.getHeight())
            &&  (f_img1.getHeight() == f_img6.getHeight()));
    
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image3Type>::raw_iterator third_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image4Type>::raw_iterator fourth_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image5Type>::raw_iterator fifth_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image6Type>::raw_iterator sixth_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1     = f_img1.end_raw();

    first_raw_iterator_type coliter1, colendunaligned1, colend1;

    second_raw_iterator_type rowiter2   = f_img2.begin_raw();
    second_raw_iterator_type coliter2;

    third_raw_iterator_type rowiter3    = f_img3.begin_raw();
    third_raw_iterator_type coliter3;

    fourth_raw_iterator_type rowiter4   = f_img4.begin_raw();
    fourth_raw_iterator_type coliter4;

    fifth_raw_iterator_type rowiter5    = f_img5.begin_raw();
    fifth_raw_iterator_type coliter5;

    sixth_raw_iterator_type rowiter6    = f_img6.begin_raw();
    sixth_raw_iterator_type coliter6;

    // calc unaligned span length
    int32_t    unaligned = (f_img1.getWidth()&0x0F);

    for (;      rowiter1 != rowend1; 
                rowiter1 += f_img1.getStride(), rowiter2 += f_img2.getStride(), 
                rowiter3 += f_img3.getStride(), rowiter4 += f_img4.getStride(), 
                rowiter5 += f_img5.getStride(), rowiter6 += f_img6.getStride())
    {
        coliter1        = rowiter1;
        colendunaligned1= rowiter1 + unaligned;
        colend1         = rowiter1 + f_img1.getWidth();

        coliter2        = rowiter2;
        coliter3        = rowiter3;
        coliter4        = rowiter4;
        coliter5        = rowiter5;
        coliter6        = rowiter6;

        for(;   coliter1 != colendunaligned1; 
                ++coliter1, ++coliter2, 
                ++coliter3, ++coliter4,
                ++coliter5, ++coliter6)
        {
            f_func(coliter1, coliter2, coliter3, coliter4, coliter5, coliter6);
        }

        for(;   coliter1 != colend1; 
                coliter1 += 16, coliter2 += 16, 
                coliter3 += 16, coliter4 += 16,
                coliter5 += 16, coliter6 += 16)
        {
            f_func( coliter1,    coliter2,    coliter3,    coliter4,    coliter5,    coliter6    );        
            f_func( coliter1+1,  coliter2+1,  coliter3+1,  coliter4+1,  coliter5+1,  coliter6+1  );    
            f_func( coliter1+2,  coliter2+2,  coliter3+2,  coliter4+2,  coliter5+2,  coliter6+2  );    
            f_func( coliter1+3,  coliter2+3,  coliter3+3,  coliter4+3,  coliter5+3,  coliter6+3  );
            f_func( coliter1+4,  coliter2+4,  coliter3+4,  coliter4+4,  coliter5+4,  coliter6+4  );    
            f_func( coliter1+5,  coliter2+5,  coliter3+5,  coliter4+5,  coliter5+5,  coliter6+5  );    
            f_func( coliter1+6,  coliter2+6,  coliter3+6,  coliter4+6,  coliter5+6,  coliter6+6  );    
            f_func( coliter1+7,  coliter2+7,  coliter3+7,  coliter4+7,  coliter5+7,  coliter6+7  );
            f_func( coliter1+8,  coliter2+8,  coliter3+8,  coliter4+8,  coliter5+8,  coliter6+8  );    
            f_func( coliter1+9,  coliter2+9,  coliter3+9,  coliter4+9,  coliter5+9,  coliter6+9  );    
            f_func( coliter1+10, coliter2+10, coliter3+10, coliter4+10, coliter5+10, coliter6+10 );    
            f_func( coliter1+11, coliter2+11, coliter3+11, coliter4+11, coliter5+11, coliter6+11 );
            f_func( coliter1+12, coliter2+12, coliter3+12, coliter4+12, coliter5+12, coliter6+12 );    
            f_func( coliter1+13, coliter2+13, coliter3+13, coliter4+13, coliter5+13, coliter6+13 );    
            f_func( coliter1+14, coliter2+14, coliter3+14, coliter4+14, coliter5+14, coliter6+14 );    
            f_func( coliter1+15, coliter2+15, coliter3+15, coliter4+15, coliter5+15, coliter6+15 );
        }
    }

}

template <class ImageType, class FunctorType>
void    vfc::for_each_pixel_backward    (ImageType& f_img, FunctorType& f_func)
{
    typedef typename TImageRawIteratorTraits<ImageType>::raw_iterator raw_iterator_type;
    
    raw_iterator_type rowiter   = f_img.getData()+f_img.getWidth()+f_img.getSize()-f_img.getStride()-1;
    raw_iterator_type rowend    = rowiter-f_img.getSize();

    raw_iterator_type coliter, colendunaligned, colend;

    // calc unaligned span length
    int32_t    unaligned = (f_img.getWidth()&0x0F);

    for (;  rowiter != rowend; 
            rowiter -= f_img.getStride())
    {
        coliter         = rowiter;
        colendunaligned = rowiter - unaligned;
        colend          = rowiter - f_img.getWidth();

        for(;   coliter != colendunaligned; 
                --coliter)
        {
            f_func(coliter);
        }

        for(;   coliter != colend; 
                coliter -= 16)
        {
            f_func(coliter);    f_func(coliter-1);  f_func(coliter-2);  f_func(coliter-3);
            f_func(coliter-4);  f_func(coliter-5);  f_func(coliter-6);  f_func(coliter-7);
            f_func(coliter-8);  f_func(coliter-9);  f_func(coliter-10); f_func(coliter-11);
            f_func(coliter-12); f_func(coliter-13); f_func(coliter-14); f_func(coliter-15);
        }
    }

}

template <class Image1Type, class Image2Type, class FunctorType>    
void    vfc::for_each_pixel_backward    (Image1Type& f_img1, Image2Type& f_img2, FunctorType& f_func)
{
    VFC_REQUIRE ((f_img1.getWidth()  == f_img2.getWidth()) 
            &&  (f_img1.getHeight() == f_img2.getHeight()));
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.getData()+f_img1.getWidth()+f_img1.getSize()-f_img1.getStride()-1;
    first_raw_iterator_type rowend1     = rowiter1-f_img1.getSize();

    first_raw_iterator_type coliter1, colendunaligned1, colend1;

    second_raw_iterator_type rowiter2   = f_img2.getData()+f_img2.getWidth()+f_img2.getSize()-f_img2.getStride()-1;
    second_raw_iterator_type coliter2;

    // calc unaligned span length
    int32_t    unaligned = (f_img1.getWidth()&0x0F);

    for (;  rowiter1 != rowend1; 
            rowiter1 -= f_img1.getStride(), rowiter2 -= f_img2.getStride())
    {
        coliter1        = rowiter1;
        colendunaligned1= rowiter1 - unaligned;
        colend1         = rowiter1 - f_img1.getWidth();

        coliter2        = rowiter2;

        for(;   coliter1 != colendunaligned1; 
                --coliter1, --coliter2)
        {
            f_func( coliter1, coliter2);
        }

        for(;   coliter1 != colend1; 
                coliter1 -= 16, coliter2 -= 16)
        {
            f_func( coliter1,    coliter2);    f_func( coliter1-1,  coliter2-1);    
            f_func( coliter1-2,  coliter2-2);  f_func( coliter1-3,  coliter2-3);
            f_func( coliter1-4,  coliter2-4);  f_func( coliter1-5,  coliter2-5);    
            f_func( coliter1-6,  coliter2-6);  f_func( coliter1-7,  coliter2-7);
            f_func( coliter1-8,  coliter2-8);  f_func( coliter1-9,  coliter2-9);    
            f_func( coliter1-10, coliter2-10); f_func( coliter1-11, coliter2-11);
            f_func( coliter1-12, coliter2-12); f_func( coliter1-13, coliter2-13);    
            f_func( coliter1-14, coliter2-14); f_func( coliter1-15, coliter2-15);
        }
    }
}

template <  vfc::int32_t SkipValue, 
            class ImageType, 
            class FunctorType>
void    vfc::for_each_nth_pixel (   ImageType& f_img, 
                                    FunctorType& f_func)
{
    //SkipValue should greater than 0
    VFC_STATIC_ASSERT(SkipValue > 0);

    typedef typename TImageRawIteratorTraits<ImageType>::raw_iterator raw_iterator_type;
    
    raw_iterator_type rowiter   = f_img.begin_raw();
    raw_iterator_type rowend    = f_img.end_raw();
    
    raw_iterator_type coliter, colend;

    // calc unaligned span length
    const int32_t rowskip = SkipValue * f_img.getStride();

    for (;  rowiter < rowend; 
            rowiter += rowskip)
    {
        coliter = rowiter;
        colend  = rowiter + f_img.getWidth();

        for(;   coliter < colend; 
                coliter += SkipValue)
        {
            f_func(coliter);
        }
    }
}

template <  vfc::int32_t SkipValue, 
            class Image1Type, class Image2Type, 
            class FunctorType>    
void    vfc::for_each_nth_pixel (   Image1Type& f_img1, Image2Type& f_img2, 
                                    FunctorType& f_func)
{
    //SkipValue should greater than 0
    VFC_STATIC_ASSERT(SkipValue > 0);

    VFC_REQUIRE((f_img1.getWidth() == f_img2.getWidth()) 
            &&  (f_img1.getHeight() == f_img2.getHeight()));
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1     = f_img1.end_raw();

    first_raw_iterator_type coliter1, colend1;
    
    const    int32_t rowskip1 = SkipValue * f_img1.getStride();

    second_raw_iterator_type rowiter2   = f_img2.begin_raw();
    second_raw_iterator_type coliter2;

    const    int32_t rowskip2 = SkipValue * f_img2.getStride();

    for (;  rowiter1 < rowend1; 
            rowiter1 += rowskip1, rowiter2 += rowskip2)
    {
        coliter1 = rowiter1;
        colend1  = rowiter1 + f_img1.getWidth();

        coliter2= rowiter2;

        for(;   coliter1 < colend1; 
                coliter1 += SkipValue, coliter2 += SkipValue)
        {
            f_func(coliter1,coliter2);
        }
    }

}

template <  vfc::int32_t SkipValue, 
            class Image1Type, class Image2Type, class Image3Type, 
            class FunctorType>
void    vfc::for_each_nth_pixel (   Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                    FunctorType& f_func)
{
    //SkipValue should be greater than 0
    VFC_STATIC_ASSERT(SkipValue > 0);
  
    VFC_REQUIRE((f_img1.getWidth()  == f_img2.getWidth())
        &&  (f_img1.getWidth()  == f_img3.getWidth())
        &&  (f_img1.getHeight() == f_img2.getHeight())
        &&  (f_img1.getHeight() == f_img3.getHeight()));
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image3Type>::raw_iterator third_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1     = f_img1.end_raw();

    first_raw_iterator_type coliter1, colend1;
    
    const int32_t rowskip1 = SkipValue * f_img1.getStride();

    second_raw_iterator_type rowiter2    = f_img2.begin_raw();
    second_raw_iterator_type coliter2;

    const int32_t rowskip2 = SkipValue * f_img2.getStride();

    third_raw_iterator_type rowiter3    = f_img3.begin_raw();
    third_raw_iterator_type coliter3;
    
    const int32_t rowskip3 = SkipValue * f_img3.getStride();

    for (;  rowiter1 < rowend1; 
            rowiter1 += rowskip1, rowiter2 += rowskip2, 
            rowiter3 += rowskip3)
    {
        coliter1= rowiter1;
        colend1    = rowiter1 + f_img1.getWidth();

        coliter2= rowiter2;
        coliter3= rowiter3;

        for(;   coliter1 < colend1; 
                coliter1 += SkipValue, coliter2 += SkipValue, 
                coliter3 += SkipValue)
        {
            f_func( coliter1, coliter2, coliter3);
        }
    }
}

template    <   vfc::int32_t SkipValue, 
                class Image1Type, class Image2Type, class Image3Type, 
                class Image4Type, 
                class FunctorType>
void    vfc::for_each_nth_pixel (   Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                    Image4Type& f_img4, 
                                    FunctorType& f_func)
{
    //SkipValue should greater than 0
    VFC_STATIC_ASSERT(SkipValue > 0);

    VFC_REQUIRE ((f_img1.getWidth()  == f_img2.getWidth())
        && (f_img1.getWidth()  == f_img3.getWidth())
        && (f_img1.getWidth()  == f_img4.getWidth())
        && (f_img1.getHeight() == f_img2.getHeight())
        && (f_img1.getHeight() == f_img3.getHeight())
        && (f_img1.getHeight() == f_img4.getHeight()));
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image3Type>::raw_iterator third_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image4Type>::raw_iterator fourth_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1        = f_img1.end_raw();

    first_raw_iterator_type coliter1, colend1;
    
    const int32_t rowskip1 = SkipValue * f_img1.getStride();

    second_raw_iterator_type rowiter2    = f_img2.begin_raw();
    second_raw_iterator_type coliter2;

    const int32_t rowskip2 = SkipValue * f_img2.getStride();

    third_raw_iterator_type rowiter3    = f_img3.begin_raw();
    third_raw_iterator_type coliter3;
    
    const int32_t rowskip3 = SkipValue * f_img3.getStride();

    fourth_raw_iterator_type rowiter4    = f_img4.begin_raw();
    fourth_raw_iterator_type coliter4;
    
    const int32_t rowskip4 = SkipValue * f_img4.getStride();

    for (;  rowiter1 < rowend1; 
            rowiter1 += rowskip1, rowiter2 += rowskip2, 
            rowiter3 += rowskip3, rowiter4 += rowskip4)
    {
        coliter1    = rowiter1;
        colend1     = rowiter1 + f_img1.getWidth();

        coliter2= rowiter2;
        coliter3= rowiter3;
        coliter4= rowiter4;

        for(;   coliter1 < colend1; 
                coliter1 += SkipValue, coliter2 += SkipValue, 
                coliter3 += SkipValue, coliter4 += SkipValue)
        {
            f_func( coliter1, coliter2, coliter3, coliter4);
        }
    }

}

template    <   class ImageType, 
                class FunctorType>
void    vfc::for_each_pixel_xy  (   ImageType&      f_img, 
                                    FunctorType&    f_func, 
                                    int32_t f_xOffset_i32, int32_t f_yOffset_i32)
{
    typedef typename TImageRawIteratorTraits<ImageType>::raw_iterator raw_iterator_type;
    
    raw_iterator_type rowiter   = f_img.begin_raw();
    raw_iterator_type rowend    = f_img.end_raw();
    
    raw_iterator_type coliter, colendunaligned, colend;

    // calc unaligned span length
    int32_t    unaligned = (f_img.getWidth()&0x0F);
    
    int32_t x = f_xOffset_i32,
            y = f_yOffset_i32;

    for (;  rowiter != rowend; 
            rowiter += f_img.getStride(), ++y)
    {
        coliter         = rowiter;
        colendunaligned = rowiter + unaligned;
        colend          = rowiter + f_img.getWidth();

        x = f_xOffset_i32;

        for(;   coliter != colendunaligned; 
                ++coliter, ++x)
        {
            f_func( x, y, coliter);
        }

        for(;   coliter != colend; 
                coliter += 16, x += 16)
        {
            f_func( x,    y, coliter);    f_func( x+1,  y, coliter+1);    
            f_func( x+2,  y, coliter+2);  f_func( x+3,  y, coliter+3);
            f_func( x+4,  y, coliter+4);  f_func( x+5,  y, coliter+5);    
            f_func( x+6,  y, coliter+6);  f_func( x+7,  y, coliter+7);
            f_func( x+8,  y, coliter+8);  f_func( x+9,  y, coliter+9);    
            f_func( x+10, y, coliter+10); f_func( x+11, y, coliter+11);
            f_func( x+12, y, coliter+12); f_func( x+13, y, coliter+13);    
            f_func( x+14, y, coliter+14); f_func( x+15, y, coliter+15);
        }
    }
    
}

template    <   class Image1Type, class Image2Type, 
                class FunctorType>
void    vfc::for_each_pixel_xy  (   Image1Type&     f_img1, Image2Type& f_img2, 
                                    FunctorType&    f_func, 
                                    int32_t f_xOffset1_i32, int32_t f_yOffset1_i32,
                                    int32_t f_xOffset2_i32, int32_t f_yOffset2_i32)
{
    VFC_REQUIRE ((f_img1.getWidth()  == f_img2.getWidth()) 
            &&  (f_img1.getHeight() == f_img2.getHeight()));
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1     = f_img1.end_raw();

    first_raw_iterator_type coliter1, colendunaligned1, colend1;

    second_raw_iterator_type rowiter2    = f_img2.begin_raw();
    second_raw_iterator_type coliter2;
 
    // calc unaligned span length
    int32_t    unaligned = (f_img1.getWidth()&0x0F);
    
    int32_t x1 = f_xOffset1_i32,
            y1 = f_yOffset1_i32,
            x2 = f_xOffset2_i32,
            y2 = f_yOffset2_i32;

    for (;  rowiter1 != rowend1; 
            rowiter1 += f_img1.getStride(), rowiter2 += f_img2.getStride(), ++y1, ++y2)
    {
        coliter1        = rowiter1;
        colendunaligned1= rowiter1 + unaligned;
        colend1         = rowiter1 + f_img1.getWidth();

        coliter2        = rowiter2;

        x1 = f_xOffset1_i32;
        x2 = f_xOffset2_i32;

        // single step for unaligned pixels
        for(;   coliter1 != colendunaligned1; 
                ++coliter1, ++coliter2,  ++x1, ++x2)
        {
            f_func(x1, y1, x2, y2, coliter1, coliter2);
        }

        // pixels are now 16 aligned
        for(;   coliter1 != colend1; 
                coliter1 += 16, coliter2 += 16, x1 += 16, x2 += 16)
        {
            f_func( x1,    y1, x2,    y2, coliter1,    coliter2);    f_func( x1+1,  y1, x2+1,  y2, coliter1+1,  coliter2+1);    
            f_func( x1+2,  y1, x2+2,  y2, coliter1+2,  coliter2+2);  f_func( x1+3,  y1, x2+3,  y2, coliter1+3,  coliter2+3);
            f_func( x1+4,  y1, x2+4,  y2, coliter1+4,  coliter2+4);  f_func( x1+5,  y1, x2+5,  y2, coliter1+5,  coliter2+5);    
            f_func( x1+6,  y1, x2+6,  y2, coliter1+6,  coliter2+6);  f_func( x1+7,  y1, x2+7,  y2, coliter1+7,  coliter2+7);
            f_func( x1+8,  y1, x2+8,  y2, coliter1+8,  coliter2+8);  f_func( x1+9,  y1, x2+9,  y2, coliter1+9,  coliter2+9);    
            f_func( x1+10, y1, x2+10, y2, coliter1+10, coliter2+10); f_func( x1+11, y1, x2+11, y2, coliter1+11, coliter2+11);
            f_func( x1+12, y1, x2+12, y2, coliter1+12, coliter2+12); f_func( x1+13, y1, x2+13, y2, coliter1+13, coliter2+13);    
            f_func( x1+14, y1, x2+14, y2, coliter1+14, coliter2+14); f_func( x1+15, y1, x2+15, y2, coliter1+15, coliter2+15);
        }
    }
    
}

template    <   class Image1Type, class Image2Type, class Image3Type,
                class FunctorType>
void    vfc::for_each_pixel_xy  (   Image1Type&     f_img1, Image2Type& f_img2, Image3Type& f_img3,
                                    FunctorType&    f_func, 
                                    int32_t f_xOffset1_i32, int32_t f_yOffset1_i32,
                                    int32_t f_xOffset2_i32, int32_t f_yOffset2_i32,
                                    int32_t f_xOffset3_i32, int32_t f_yOffset3_i32)
{
    VFC_REQUIRE (    (f_img1.getWidth()  == f_img2.getWidth())
        &&  (f_img1.getWidth()  == f_img3.getWidth())
        &&  (f_img1.getHeight() == f_img2.getHeight())
        &&  (f_img1.getHeight() == f_img3.getHeight())    );
    typedef typename TImageRawIteratorTraits<Image1Type>::raw_iterator first_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image2Type>::raw_iterator second_raw_iterator_type;
    typedef typename TImageRawIteratorTraits<Image3Type>::raw_iterator third_raw_iterator_type;
    
    first_raw_iterator_type rowiter1    = f_img1.begin_raw();
    first_raw_iterator_type rowend1     = f_img1.end_raw();

    first_raw_iterator_type coliter1, colendunaligned1, colend1;

    second_raw_iterator_type rowiter2    = f_img2.begin_raw();
    second_raw_iterator_type coliter2;
 
    third_raw_iterator_type rowiter3    = f_img3.begin_raw();
    third_raw_iterator_type coliter3;

    // calc unaligned span length
    int32_t    unaligned = (f_img1.getWidth()&0x0F);
    
    int32_t x1 = f_xOffset1_i32,
            y1 = f_yOffset1_i32,
            x2 = f_xOffset2_i32,
            y2 = f_yOffset2_i32,
            x3 = f_xOffset3_i32,
            y3 = f_yOffset3_i32;

    for (;  rowiter1 != rowend1; 
            rowiter1 += f_img1.getStride(), rowiter2 += f_img2.getStride(), rowiter3 += f_img3.getStride(),
            ++y1, ++y2, ++y3)
    {
        coliter1        = rowiter1;
        colendunaligned1= rowiter1 + unaligned;
        colend1         = rowiter1 + f_img1.getWidth();

        coliter2        = rowiter2;
        coliter3        = rowiter3;

        x1 = f_xOffset1_i32;
        x2 = f_xOffset2_i32;
        x3 = f_xOffset3_i32;

        // single step for unaligned pixels
        for(;   coliter1 != colendunaligned1; 
                ++coliter1, ++coliter2, ++coliter3, 
                ++x1, ++x2, ++x3)
        {
            f_func(x1, y1, x2, y2, x3, y3, coliter1, coliter2, coliter3);
        }

        // pixels are now 16 aligned
        for(;   coliter1 != colend1; 
                coliter1 += 16, coliter2 += 16, coliter3 += 16,
                x1 += 16, x2 += 16, x3 += 16)
        {
            f_func( x1,    y1, x2,    y2, x3,    y3,  coliter1,    coliter2,    coliter3);    
            f_func( x1+1,  y1, x2+1,  y2, x3+1,  y3,  coliter1+1,  coliter2+1,  coliter3+1);    
            f_func( x1+2,  y1, x2+2,  y2, x3+2,  y3,  coliter1+2,  coliter2+2,  coliter3+2);  
            f_func( x1+3,  y1, x2+3,  y2, x3+3,  y3,  coliter1+3,  coliter2+3,  coliter3+3);
            f_func( x1+4,  y1, x2+4,  y2, x3+4,  y3,  coliter1+4,  coliter2+4,  coliter3+4);  
            f_func( x1+5,  y1, x2+5,  y2, x3+5,  y3,  coliter1+5,  coliter2+5,  coliter3+5);    
            f_func( x1+6,  y1, x2+6,  y2, x3+6,  y3,  coliter1+6,  coliter2+6,  coliter3+6);  
            f_func( x1+7,  y1, x2+7,  y2, x3+7,  y3,  coliter1+7,  coliter2+7,  coliter3+7);
            f_func( x1+8,  y1, x2+8,  y2, x3+8,  y3,  coliter1+8,  coliter2+8,  coliter3+8);  
            f_func( x1+9,  y1, x2+9,  y2, x3+9,  y3,  coliter1+9,  coliter2+9,  coliter3+9);    
            f_func( x1+10, y1, x2+10, y2, x3+10, y3,  coliter1+10, coliter2+10, coliter3+10); 
            f_func( x1+11, y1, x2+11, y2, x3+11, y3,  coliter1+11, coliter2+11, coliter3+11);
            f_func( x1+12, y1, x2+12, y2, x3+12, y3,  coliter1+12, coliter2+12, coliter3+12); 
            f_func( x1+13, y1, x2+13, y2, x3+13, y3,  coliter1+13, coliter2+13, coliter3+13);    
            f_func( x1+14, y1, x2+14, y2, x3+14, y3,  coliter1+14, coliter2+14, coliter3+14); 
            f_func( x1+15, y1, x2+15, y2, x3+15, y3,  coliter1+15, coliter2+15, coliter3+15);
        }
    }
    
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
// $Log: vfc_image_algorithm.inl  $
// Revision 1.25 2007/11/26 16:55:09MEZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
// - fixed constness bug (mantis1961)
// Revision 1.24 2007/10/31 13:29:22CET vmr1kor 
// missing parenthesis in VFC_REQUIRE() are added
// Mantis Id :- 0001717 
// Revision 1.23 2007/08/02 19:15:56IST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
// - added conditional doxygen documentation generation of vfc::intern (mantis1758)
// Revision 1.22 2007/05/08 17:40:54CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
// - fixed bug in typedefs (mantis1638)
// Revision 1.21 2007/04/24 15:42:33CEST dkn2kor 
// - VFC_REQUIRE precondition checks used instead of if statement.  
//   the function interface has changed.  returns void instead of bool (mantis1578)
//   
// Revision 1.20 2007/04/02 19:02:46IST Koenig Matthias (CR/AEM5) (kon3hi) 
// Added for_each_pixel_xy for two and three images (MANTIS 0001389)
// Revision 1.19 2007/01/29 14:47:31CET ZVH2HI 
// - replaced header + footer with c++ version
// - added for_each_pixel() with 5 and 6 images (mantis1396)
// Revision 1.18 2006/11/16 14:41:15CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
// - replaced tabs with 4 spaces (mantis1294)
// Revision 1.17 2006/08/10 14:23:02CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
// -added TImageRawIteratorTraits<> for const-correctness (mantis1134)
// -changed code to begin_raw() and end_raw()
// -TODO: fix documentation for implicit interface
// Revision 1.16 2006/03/07 15:33:26CET Dilipkrishna Natesan (RBIN/EAE3 AE-DA/ESA3) * (DIN1LR) 
// -removed colendunaligned1 unused varaible (mantis1021)
// Revision 1.15 2006/03/07 15:17:59CET Dilipkrishna Natesan (RBIN/EAE3 AE-DA/ESA3) * (DIN1LR) 
// -added static assert for invalid SkipValue in for_each_nth_pixel function (mantis1020)
// Revision 1.14 2006/02/01 17:24:36CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added for_each_pixel_backward() function for two images
// Revision 1.13 2006/02/01 16:27:04CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added for_each_pixel_backward()
// Revision 1.12 2005/12/01 17:27:02CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -cosmetics
// Revision 1.11 2005/12/01 16:58:39CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added for_each_pixel_xy()
// Revision 1.10 2005/11/16 11:46:02CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -moved for_each_nmth_pixel() to vfc_image_algorithm_ex.hpp (function ist still experimental and only used for RSR)
// Revision 1.9 2005/11/15 09:53:09CET Alaa El-Din Omar (AE-DA/ESA3-Hi) * (ALO2HI) 
// - added function for_each_nmth_pixel to allow writing output image with different size than input image (packed subampled output)
// Revision 1.8 2005/11/01 10:31:06CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added four image for_each_...
// -fixed bug in for_each_nth_pixel()
// Revision 1.7 2005/10/12 15:20:30CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added missing namespace
// Revision 1.6 2005/10/11 15:20:23CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -replaced != with <
// Revision 1.5 2005/10/11 15:15:40CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -experimental nth pixel algo
// Revision 1.4 2005/10/11 15:02:41CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -fixed wrong loop statement
// Revision 1.3 2005/10/06 17:06:03CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// Initial revision
// Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
// Revision 1.2 2005/09/13 17:14:48CEST zvh2hi 
// -several changes
//=============================================================================
