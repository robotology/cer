/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef CER_DEV_FACEDISPLAY_INTERFACE_H_
#define CER_DEV_FACEDISPLAY_INTERFACE_H_

#include <yarp/os/Vocab.h>
#include <yarp/sig/Image.h>

// VOCABS
#define VOCAB_SET           yarp::os::createVocab32('s','e','t')
#define VOCAB_GET           yarp::os::createVocab32('g','e','t')

#define VOCAB_FACE          yarp::os::createVocab32('f','a','c','e')
#define VOCAB_FILE          yarp::os::createVocab32('f','i','l','e')
#define VOCAB_MOVE          yarp::os::createVocab32('m','o','v','e')
#define VOCAB_IMAGE         yarp::os::createVocab32('i','m','g')

#define VOCAB_FACE_HAPPY    yarp::os::createVocab32('h','a','p')
#define VOCAB_FACE_SAD      yarp::os::createVocab32('s','a','d')
#define VOCAB_FACE_WARNING  yarp::os::createVocab32('w','a','r','n')

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

    virtual bool setImage(yarp::sig::Image  img) = 0;
    virtual bool getImage(yarp::sig::Image *img) = 0;
};

#endif // CER_DEV_FACEDISPLAY_INTERFACE_H_
