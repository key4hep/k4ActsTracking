#pragma once

// edm4hep
#include <edm4hep/MCParticle.h>
#include <edm4hep/Track.h>
#include <edm4hep/MutableTrack.h>
#include <edm4hep/TrackState.h>
#include <edm4hep/TrackCollection.h>

// Gaudi
#include <GaudiKernel/MsgStream.h>
#include <GaudiKernel/ServiceHandle.h>
#include <GaudiKernel/StatusCode.h>
#include <GaudiKernel/IEvtSelector.h>
#include <GaudiKernel/IDataProviderSvc.h>
#include <GaudiKernel/IDataManagerSvc.h>
#include <GaudiKernel/SmartDataPtr.h>

// ACTS
#include <Acts/EventData/TrackParameters.hpp>
#include "Acts/EventData/ParticleHypothesis.hpp"
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFitting/KalmanFitter.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>

// ACTSTracking
#include "k4ActsTracking/SourceLink.hxx"

namespace ACTSTracking {

using TrackResult = Acts::TrackContainer<Acts::VectorTrackContainer,
                                         Acts::VectorMultiTrajectory,
                                         std::shared_ptr>::TrackProxy;

//! Get path to a resource file
/**
 * Get absolute file of a file `inpath` by looking in the following places:
 *  - `inpath` to the current working directory
 *  - `ACTSTRACKING_SOURCEDIR/inpath`
 *  - `ACTSTRACKING_DATADIR/inpath`
 *
 * If the files is not found at any location, then `inpath` is returned.
 * If `path` starts with a /, then it is returned directly.
 *
 * \parm inpath File to find.
 *
 * \return Absolute path to file.
 */
std::string findFile(const std::string& inpath);

//! Convert ACTS KF result to edm4hep track class
/**
 * Converted properties are:
 *  - goodness of fit (chi2, ndf)
 *  - associated hits
 *  - track states at IP
 *
 * \param fitOutput KF fit result
 * \param magneticField magnetic field at different locations in the detector
 * \param magCache cache to help with magnetic field lookup
 *
 * \return Track with equivalent parameters of the ACTS track
 */
edm4hep::MutableTrack* ACTS2edm4hep_track(
    const TrackResult& fitter_res,
    std::shared_ptr<Acts::MagneticFieldProvider> magneticField,
    Acts::MagneticFieldProvider::Cache& magCache);

//! Convert ACTS track state class to edm4hep class
/**
 * \param location Location where the track state is defined (ie: `AtIP`)
 * \param params ACTS track state parameters
 * \params Bz magnetic field at location of track state [Tesla]
 *
 * \return Track state with equivalent parameters of the ACTS track
 */
edm4hep::TrackState* ACTS2edm4hep_trackState(
    int location, const Acts::BoundTrackParameters& params, double Bz);
//! Helper Method for ACTS2edm4hep_trackState
edm4hep::TrackState* ACTS2edm4hep_trackState(int location,
                                          const Acts::BoundVector& value,
                                          const Acts::BoundMatrix& cov,
                                          double Bz);

//! Get particle hypothesis in ACTS format 
/**
 * \param MCParticle
 *
 * \return Particle Hypothesis based on MCParticle PDG
 */
Acts::ParticleHypothesis convertParticle(const edm4hep::MCParticle mcParticle);

}  // namespace ACTSTracking

