/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef CER_CIRCULAR_BUFFER_H
#define CER_CIRCULAR_BUFFER_H


//----------------------------------------------------------------------------------
template <class T>
class CircularBuffer
{
private:
    int         maxsize;
    int         start;
    int         end;
    T          *elems;

public:
    inline bool isFull()
    {
        return (end + 1) % maxsize == start;
    }

    inline const T* getRawData()
    {
        return elems;
    }

    inline bool isEmpty()
    {
        return end == start;
    }

    inline int size()
    {
        int i;
        if (end>start)
            i = end-start;
        else if (end==start)
            i = 0;
        else
            i = maxsize - start + end;
        return i;
    }

    inline T read()
    {
        if (end == start)
        {
            printf ("ERROR: buffer underrun!\n");
        }
        T elem = elems[start];
        start = (start + 1) % maxsize;
        return elem;
    }

    inline void write(T elem)
    {
        elems[end] = elem;
        end = (end + 1) % maxsize;
        if (end == start)
        {
            printf ("ERROR: buffer ovverrun!\n");
            start = (start + 1) % maxsize; // full, overwrite
        }
    }

    inline T* getReadRaw()
    {
        if (end == start)
        {
            printf ("ERROR: buffer underrun!\n");
        }
        int idx = start;
        start = (start + 1) % maxsize;
        return &elems[idx];
    }

    inline T* getWriteRaw()
    {
        if (end == start)
        {
            printf ("ERROR: buffer underrun!\n");
        }
        int idx = end;
        end = (end + 1) % maxsize;
        return &elems[idx];
    }

    inline unsigned int getMaxSize()
    {
        return maxsize;
    }

    inline void clear()
    {
        start = 0;
        end   = 0;
    }

    CircularBuffer(int bufferSize)
    {
        maxsize  = bufferSize + 1;
        start = 0;
        end   = 0;
        elems = new T[maxsize];
    }

    ~CircularBuffer()
    {
        delete[] elems;
    }
};

//----------------------------------------------------------------------------------

#endif  // CER_CIRCULAR_BUFFER_H
