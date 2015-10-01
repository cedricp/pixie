## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

SET(LIBRARY_PATHS
  /usr/lib /usr/local/lib /opt/local/lib ${OPENEXR_ROOT}/lib)

FIND_PATH(OPENEXR_INCLUDE_DIR OpenEXR/ImfHeader.h
  ${OPENEXR_ROOT}/include
  /usr/include       /usr/include/
  /usr/local/include /usr/local/include/ 
  /opt/local/include /opt/local/include/ 
  )

FIND_LIBRARY(OPENEXR_HALF_LIBRARY      NAMES Half                       PATHS ${LIBRARY_PATHS})
FIND_LIBRARY(OPENEXR_IEX_LIBRARY       NAMES Iex${OPENEXR_SUFFIX}       PATHS ${LIBRARY_PATHS})
FIND_LIBRARY(OPENEXR_IEXMATH_LIBRARY   NAMES IexMath${OPENEXR_SUFFIX}   PATHS ${LIBRARY_PATHS})
FIND_LIBRARY(OPENEXR_IMATH_LIBRARY     NAMES Imath${OPENEXR_SUFFIX}     PATHS ${LIBRARY_PATHS})
FIND_LIBRARY(OPENEXR_ILMIMF_LIBRARY    NAMES IlmImf${OPENEXR_SUFFIX}    PATHS ${LIBRARY_PATHS})
FIND_LIBRARY(OPENEXR_ILMTHREAD_LIBRARY NAMES IlmThread${OPENEXR_SUFFIX} PATHS ${LIBRARY_PATHS})

SET(OPENEXR_INCLUDE_DIRS ${OPENEXR_INCLUDE_DIR} ${OPENEXR_INCLUDE_DIR}/OpenEXR)
SET(OPENEXR_LIBRARIES ${OPENEXR_ILMIMF_LIBRARY} ${OPENEXR_IMATH_LIBRARY} ${OPENEXR_HALF_LIBRARY} ${OPENEXR_IEX_LIBRARY} ${OPENEXR_IEXMATH_LIBRARY} ${OPENEXR_ILMTHREAD_LIBRARY})

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OPENEXR DEFAULT_MSG OPENEXR_INCLUDE_DIR OPENEXR_HALF_LIBRARY OPENEXR_IEX_LIBRARY OPENEXR_IMATH_LIBRARY OPENEXR_ILMIMF_LIBRARY OPENEXR_ILMTHREAD_LIBRARY)

MARK_AS_ADVANCED(
  OPENEXR_INCLUDE_DIR
  OPENEXR_HALF_LIBRARY
  OPENEXR_IEX_LIBRARY
  OPENEXR_IMATH_LIBRARY
  OPENEXR_ILMIMF_LIBRARY
  OPENEXR_ILMTHREAD_LIBRARY
)
