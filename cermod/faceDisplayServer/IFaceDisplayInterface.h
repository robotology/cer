/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef CER_DEV_FACEDISPLAY_INTERFACE_H_
#define CER_DEV_FACEDISPLAY_INTERFACE_H_

#include <yarp/os/Vocab.h>

// VOCABS
#define VOCAB_FACE          VOCAB4('f','a','c','e')
#define VOCAB_FILE          VOCAB4('f','i','l','e')

#define VOCAB_FACE_HAPPY    VOCAB3('h','a','p')
#define VOCAB_FACE_SAD      VOCAB3('s','a','d')
#define VOCAB_FACE_WARNING  VOCAB4('w','a','r','n')

namespace cer {
    namespace dev {
        class IFaceDisplay;
    }
}


/**
 * Interface a the Facial Display capable of showing images
 * and a predefined set of face expressions.
 */
class cer::dev::IFaceDisplay
{
public:
    virtual bool setFaceExpression(int faceId) = 0;
    virtual bool getFaceExpression(int *faceId) = 0;

    virtual bool setImageFile(std::string fileName) = 0;
    virtual bool getImageFile(std::string &fileName) = 0;
};

#endif // CER_DEV_FACEDISPLAY_INTERFACE_H_
