// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file SampleIpcAdapter.cpp
 * @brief Sample for the IPC Adapter QCNode, runnable with QCNodeSampleApp (-t IpcAdapter).
 *
 * The sample creates the IPC Adapter node through QCNodeFactory with one Flow and calls its
 * ProcessFrameDescriptor() from a worker thread, with a frame descriptor holding the Contract
 * descriptor. The ROS Bridge (ros_ipc_bridge) must be running.
 *
 * HCM to ROS (direction 0), for any Contract the Adapter maps: it sends every frame another
 * sample, such as StreamProbe, publishes on input_topic, as it takes it.
 * ROS to HCM (direction 1): it fetches the oldest waiting ROS message into each frame another
 * sample publishes on input_topic, which owns the frames, and publishes the filled frame on
 * output_topic. It fetches at once while messages wait and again after kNoDataWait when none
 * waits.
 *
 * It prints the node state after each lifecycle call, the configuration read back through
 * GetConfigurationIfs() after Initialize(), and the monitoring and frame counts after Stop(). The
 * monitoring line gives the frames per second over the last kRateWindow frames. With log_file, it
 * writes what it prints to that file too, each line after the CLOCK_MONOTONIC time and the sample
 * name, and every kLogPeriod a monitoring line of the frame, error and no-data counts, the
 * average latency and the fps.
 *
 * Attributes:
 *   adapter_config: (optional) a file with the Adapter's static config; it replaces the six
 *                   Flow attributes below. Without an "id", the sample's node id is used
 *   contract_type : (optional) a mapped Contract, default qrb.contract.sensor.v1.PointCloud2
 *   direction     : (optional) 0: HCM to ROS (default), 1: ROS to HCM
 *   ros_topic     : (optional) default /qrb_ros_adapter/sample/<name>, the name for the
 *                   Contract: points (PointCloud2), pose (PoseWithCovarianceStamped), odom,
 *                   cmd_vel (TwistStamped), tf, image, joint_state, pose_stamped, pose_array,
 *                   vector3, detections_2d, detections_3d, imu
 *   reliability   : (optional) reliable or best_effort, default the ROS default
 *   durability    : (optional) volatile or transient_local, default the ROS default
 *   depth         : (optional) 1..32, default the ROS default
 *   input_topic   : (required) the topic of the frames to send (direction 0) or to fill
 *                   (direction 1)
 *   output_topic  : (required for direction 1) the topic to publish the filled frames on
 *   log_file      : (optional) a text log file, default none
 */

#include "ContractFrame.hpp"
#include "QC/Node/Ifs/QCNodeFactory.hpp"
#include "QC/Node/NodeFrameDescriptor.hpp"
#include "QC/sample/DataBroker.hpp"
#include "QC/sample/SampleIF.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_node.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

namespace QC
{
namespace sample
{

using namespace QC::Node;

static const char *StateName( QCObjectState_e state )
{
    switch ( state )
    {
        case QC_OBJECT_STATE_INITIAL:
            return "INITIAL";
        case QC_OBJECT_STATE_INITIALIZING:
            return "INITIALIZING";
        case QC_OBJECT_STATE_READY:
            return "READY";
        case QC_OBJECT_STATE_STARTING:
            return "STARTING";
        case QC_OBJECT_STATE_RUNNING:
            return "RUNNING";
        case QC_OBJECT_STATE_STOPING:
            return "STOPING";
        case QC_OBJECT_STATE_ERROR:
            return "ERROR";
        default:
            return "OTHER";
    }
}

/** Frames per second over the last @p Window frames. */
template<size_t Window>
class FrameRate
{
public:
    void Clear()
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        m_times.clear();
    }

    void Add()
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        if ( Window == m_times.size() )
        {
            m_times.pop_front();
        }
        m_times.push_back( std::chrono::steady_clock::now() );
    }

    double Get() const
    {
        std::lock_guard<std::mutex> lock( m_mutex );
        if ( m_times.size() < 2 )
        {
            return 0.0;
        }
        const double seconds =
                std::chrono::duration<double>( m_times.back() - m_times.front() ).count();
        return ( seconds > 0.0 ) ? ( m_times.size() - 1 ) / seconds : 0.0;
    }

private:
    mutable std::mutex m_mutex;
    std::deque<std::chrono::steady_clock::time_point> m_times;
};

/** printf() into a string. */
__attribute__( ( format( printf, 1, 2 ) ) ) static std::string Text( const char *format, ... )
{
    va_list args;
    va_start( args, format );
    va_list measure;
    va_copy( measure, args );
    const int size = vsnprintf( nullptr, 0, format, measure );
    va_end( measure );
    std::string text( ( size > 0 ) ? size : 0, '\0' );
    if ( size > 0 )
    {
        (void) vsnprintf( &text[0], text.size() + 1, format, args );
    }
    va_end( args );
    return text;
}

class SampleIpcAdapter : public SampleIF
{
public:
    SampleIpcAdapter() = default;
    ~SampleIpcAdapter() = default;

    QCStatus_e Init( std::string name, SampleConfig_t &config ) override;
    QCStatus_e Start() override;
    QCStatus_e Stop() override;
    QCStatus_e Deinit() override;

private:
    QCStatus_e LoadAdapterConfig( const std::string &path, std::string &config );
    std::string DefaultTopic() const;
    std::string MakeAdapterConfig( SampleConfig_t &config );
    QCStatus_e Process( ContractFrame &frame );
    bool GetMonitor( qrb::adapter::HcmIpcAdapterMonitoringData &monitor );
    /** Print @p line, and write it to the log; with @p logOnly, only write it there. */
    void Print( const std::string &line, bool logOnly = false );
    void PrintState( const char *call, QCStatus_e ret );
    void PrintConfig();
    /**
     * Print the monitoring line; with @p logOnly, write it to the log without the session and the
     * last, min and max latency.
     */
    void PrintMonitor( bool logOnly );
    void SendFromTopic();
    void FillFromTopic();
    void ThreadMain();
    void LogMain();

private:
    std::unique_ptr<QCNodeIfs> m_node;
    std::string m_contract;
    bool m_isHcmToRos = true;
    uint32_t m_bufferId = 0;
    std::string m_inputTopic;
    std::string m_outputTopic;

    DataSubscriber<ContractFrameRef_t> m_sub;
    DataPublisher<ContractFrameRef_t> m_pub;

    std::thread m_thread;
    std::atomic<bool> m_stop{ false };
    uint64_t m_noDataCount = 0;
    uint64_t m_frames = 0;
    uint64_t m_errorCount = 0;

    static constexpr size_t kRateWindow = 10000;
    static constexpr auto kLogPeriod = std::chrono::seconds( 10 );
    FrameRate<kRateWindow> m_rate;
    FILE *m_log = nullptr;
    std::thread m_logThread;
    std::mutex m_logMutex;
    std::condition_variable m_logWake;

    static constexpr const char *kPointCloudContract = "qrb.contract.sensor.v1.PointCloud2";
    static constexpr const char *kImageContract = "qrb.contract.sensor.v1.ImageLayout";
    static constexpr const char *kPoseContract =
            "qrb.contract.geometry.v1.PoseWithCovarianceStamped";
    static constexpr const char *kDetection2DContract = "qrb.contract.vision.v1.Detection2DArray";
    static constexpr const char *kDetection3DContract = "qrb.contract.vision.v1.Detection3DArray";
    static constexpr const char *kImuContract = "qrb.contract.sensor.v1.Imu";
    static constexpr uint64_t kMaxReportedErrors = 10;
    static constexpr uint32_t kReceiveTimeoutMs = 100;
    // How long to wait before fetching again when no ROS message waits.
    static constexpr auto kNoDataWait = std::chrono::milliseconds( 1 );
};

QCStatus_e SampleIpcAdapter::LoadAdapterConfig( const std::string &path, std::string &config )
{
    std::ifstream in( path );
    if ( !in )
    {
        QC_ERROR( "cannot read adapter_config %s", path.c_str() );
        return QC_STATUS_BAD_ARGUMENTS;
    }
    std::ostringstream text;
    text << in.rdbuf();
    config = text.str();

    // The Adapter checks the config; the sample only reads what it needs from it.
    DataTree dt;
    DataTree node;
    std::string errors;
    if ( QC_STATUS_OK != dt.Load( config, errors ) )
    {
        QC_ERROR( "adapter_config %s is not valid JSON: %s", path.c_str(), errors.c_str() );
        return QC_STATUS_BAD_ARGUMENTS;
    }
    if ( QC_STATUS_OK != dt.Get( "static", node ) )
    {
        QC_ERROR( "adapter_config %s has no \"static\" object", path.c_str() );
        return QC_STATUS_BAD_ARGUMENTS;
    }
    // Without an "id", the file takes the sample's node id; an "id" in the file wins.
    if ( !dt.Exists( "static.id" ) )
    {
        dt.Set<uint32_t>( "static.id", m_nodeId.id );
        config = dt.Dump();
    }
    m_contract = node.Get<std::string>( "contract_type", "" );
    m_isHcmToRos = ( 0 == node.Get<uint32_t>( "direction", 0 ) );
    std::vector<DataTree> map;
    if ( ( QC_STATUS_OK == node.Get( "globalBufferIdMap", map ) ) && ( false == map.empty() ) )
    {
        m_bufferId = map[0].Get<uint32_t>( "id", 0 );
    }

    return QC_STATUS_OK;
}

std::string SampleIpcAdapter::DefaultTopic() const
{
    // The default topic name of each Contract the Adapter maps.
    static constexpr struct
    {
        const char *contract;
        const char *topic;
    } kDefaultTopics[] = {
            { kPointCloudContract, "points" },
            { kPoseContract, "pose" },
            { "qrb.contract.nav.v1.Odometry", "odom" },
            { "qrb.contract.geometry.v1.TwistStamped", "cmd_vel" },
            { "qrb.contract.tf2.v1.TFMessage", "tf" },
            { kImageContract, "image" },
            { "qrb.contract.sensor.v1.JointState", "joint_state" },
            { "qrb.contract.geometry.v1.PoseStamped", "pose_stamped" },
            { "qrb.contract.geometry.v1.PoseArray", "pose_array" },
            { "qrb.contract.geometry.v1.Vector3Stamped", "vector3" },
            { kDetection2DContract, "detections_2d" },
            { kDetection3DContract, "detections_3d" },
            { kImuContract, "imu" },
    };
    std::string topic = "/qrb_ros_adapter/sample/";
    for ( const auto &entry : kDefaultTopics )
    {
        if ( m_contract == entry.contract )
        {
            return topic + entry.topic;
        }
    }

    // Another Contract: its message name in snake_case, e.g. Odometry: odometry.
    std::string message = m_contract.substr( m_contract.rfind( '.' ) + 1 );
    bool first = true;
    for ( char c : message )
    {
        if ( ( c >= 'A' ) && ( c <= 'Z' ) )
        {
            if ( !first )
            {
                topic += '_';
            }
            c = (char) ( c - 'A' + 'a' );
        }
        topic += c;
        first = false;
    }

    return topic;
}

std::string SampleIpcAdapter::MakeAdapterConfig( SampleConfig_t &config )
{
    std::string defaultTopic = DefaultTopic();
    DataTree dt;
    dt.Set<std::string>( "static.name", m_name );
    dt.Set<uint32_t>( "static.id", m_nodeId.id );
    dt.Set<std::string>( "static.contract_type", m_contract );
    dt.Set<uint32_t>( "static.direction", m_isHcmToRos ? 0 : 1 );
    dt.Set<std::string>( "static.ros_topic", Get( config, "ros_topic", defaultTopic ) );
    std::string reliability = Get( config, "reliability", "" );
    std::string durability = Get( config, "durability", "" );
    if ( !reliability.empty() )
    {
        dt.Set<std::string>( "static.qos_profile.reliability", reliability );
    }
    if ( !durability.empty() )
    {
        dt.Set<std::string>( "static.qos_profile.durability", durability );
    }
    // A given depth goes in as it is, 0 too, for the config parser to check.
    if ( config.end() != config.find( "depth" ) )
    {
        dt.Set<uint32_t>( "static.qos_profile.depth", Get( config, "depth", (uint32_t) 0 ) );
    }

    return dt.Dump();
}

QCStatus_e SampleIpcAdapter::Init( std::string name, SampleConfig_t &config )
{
    QCStatus_e ret = SampleIF::Init( name, QC_NODE_TYPE_IPC_ADAPTER );
    std::string adapterConfig;

    if ( QC_STATUS_OK == ret )
    {
        std::string log = Get( config, "log_file", "" );
        if ( !log.empty() )
        {
            m_log = fopen( log.c_str(), "w" );
            if ( nullptr == m_log )
            {
                QC_ERROR( "cannot open log %s", log.c_str() );
                ret = QC_STATUS_BAD_ARGUMENTS;
            }
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        std::string path = Get( config, "adapter_config", "" );
        if ( !path.empty() )
        {
            ret = LoadAdapterConfig( path, adapterConfig );
        }
        else
        {
            m_contract = Get( config, "contract_type", kPointCloudContract );
            uint32_t direction = Get( config, "direction", (uint32_t) 0 );
            if ( direction > 1 )
            {
                QC_ERROR( "direction must be 0 (HCM to ROS) or 1 (ROS to HCM): %u", direction );
                ret = QC_STATUS_BAD_ARGUMENTS;
            }
            m_isHcmToRos = ( 0 == direction );
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        if ( nullptr == qrb::adapter::FindHcmContractBinding( m_contract ) )
        {
            QC_ERROR( "contract_type is not mapped by the Adapter: %s", m_contract.c_str() );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }
        m_inputTopic = Get( config, "input_topic", "" );
        m_outputTopic = Get( config, "output_topic", "" );
        if ( m_inputTopic.empty() || ( m_isHcmToRos != m_outputTopic.empty() ) )
        {
            QC_ERROR( "direction %d takes %s", m_isHcmToRos ? 0 : 1,
                      m_isHcmToRos ? "input_topic and no output_topic"
                                   : "input_topic and output_topic" );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        ret = QCNodeFactory::CreateNode( QC_NODE_TYPE_IPC_ADAPTER, m_node );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to create the IPC Adapter node: %d", ret );
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        QCNodeInit_t nodeInit;
        nodeInit.config = adapterConfig.empty() ? MakeAdapterConfig( config ) : adapterConfig;
        ret = m_node->Initialize( nodeInit );
        PrintState( "Initialize", ret );
        if ( QC_STATUS_OK == ret )
        {
            PrintConfig();
        }
        else
        {
            // NO_RESOURCE: the ROS Bridge is not running.
            QC_ERROR( "failed to initialize the IPC Adapter node with %s: %d",
                      nodeInit.config.c_str(), ret );
        }
    }

    if ( ( QC_STATUS_OK == ret ) && !m_inputTopic.empty() )
    {
        // FIFO, as deep as any pool.
        ret = m_sub.Init( name, m_inputTopic, ContractFramePool::kMaxSize, false );
    }

    if ( ( QC_STATUS_OK == ret ) && !m_outputTopic.empty() )
    {
        ret = m_pub.Init( name, m_outputTopic );
    }

    return ret;
}

QCStatus_e SampleIpcAdapter::Start()
{
    QCStatus_e ret = m_node->Start();

    PrintState( "Start", ret );
    if ( QC_STATUS_OK == ret )
    {
        m_stop = false;
        m_rate.Clear();
        m_thread = std::thread( &SampleIpcAdapter::ThreadMain, this );
        if ( nullptr != m_log )
        {
            m_logThread = std::thread( &SampleIpcAdapter::LogMain, this );
        }
    }
    else
    {
        QC_ERROR( "failed to start the IPC Adapter node: %d", ret );
    }

    return ret;
}

bool SampleIpcAdapter::GetMonitor( qrb::adapter::HcmIpcAdapterMonitoringData &monitor )
{
    uint32_t size = sizeof( monitor );
    return QC_STATUS_OK == m_node->GetMonitoringIfs().Place( &monitor, size );
}

void SampleIpcAdapter::Print( const std::string &line, bool logOnly )
{
    if ( !logOnly )
    {
        (void) printf( "%s %s\n", m_name.c_str(), line.c_str() );
    }
    if ( nullptr != m_log )
    {
        timespec now{};
        (void) clock_gettime( CLOCK_MONOTONIC, &now );
        (void) fprintf( m_log, "[%lld.%09ld] [%s]: %s\n", (long long) now.tv_sec,
                        (long) now.tv_nsec, m_name.c_str(), line.c_str() );
        (void) fflush( m_log );
    }
}

void SampleIpcAdapter::PrintState( const char *call, QCStatus_e ret )
{
    Print( Text( "%s returned %d, state %s", call, (int) ret, StateName( m_node->GetState() ) ) );
}

void SampleIpcAdapter::PrintConfig()
{
    QCNodeConfigIfs &configIfs = m_node->GetConfigurationIfs();
    const QCNodeConfigBase_t &base = configIfs.Get();
    Print( Text( "config: name=%s, type=%d, id=%u", base.nodeId.name.c_str(),
                 (int) base.nodeId.type, (unsigned) base.nodeId.id ) );
    auto *config = dynamic_cast<qrb::adapter::HcmIpcAdapterNodeConfig *>( &configIfs );
    if ( nullptr != config )
    {
        const qrb::adapter::HcmIpcAdapterConfig &adapter = config->Adapter();
        const qrb::adapter::FlowConfig &flow = adapter.flowConfig;
        Print( Text( "flow: contract_type=%s, direction=%lld, ros_topic=%s, reliability=%s, "
                     "durability=%s, depth=%u, global_buffer_id=%u",
                     flow.contractTypeName.c_str(),
                     (long long) qrb::adapter::ToDirectionCode( flow.direction ),
                     flow.rosTopic.c_str(), qrb::adapter::ToString( flow.qosProfile.reliability ),
                     qrb::adapter::ToString( flow.qosProfile.durability ), flow.qosProfile.depth,
                     adapter.globalBufferId ) );
    }
}

void SampleIpcAdapter::PrintMonitor( bool logOnly )
{
    qrb::adapter::HcmIpcAdapterMonitoringData monitor{};
    if ( GetMonitor( monitor ) )
    {
        std::string line = Text(
                "monitor: frameCount=%llu, errorCount=%llu, averageLatencyNs=%llu, "
                "noDataCount=%llu, fps=%.2f",
                (unsigned long long) monitor.frameCount, (unsigned long long) monitor.errorCount,
                (unsigned long long) monitor.averageLatencyNs,
                (unsigned long long) monitor.noDataCount, m_rate.Get() );
        if ( !logOnly )
        {
            line += Text( ", sessionConnected=%u, lastLatencyNs=%llu, minLatencyNs=%llu, "
                          "maxLatencyNs=%llu",
                          (unsigned) monitor.sessionConnected,
                          (unsigned long long) monitor.lastLatencyNs,
                          (unsigned long long) monitor.minLatencyNs,
                          (unsigned long long) monitor.maxLatencyNs );
        }
        Print( line, logOnly );
    }
}

QCStatus_e SampleIpcAdapter::Process( ContractFrame &frame )
{
    NodeFrameDescriptor frameDesc( m_bufferId + 1 );
    (void) frameDesc.SetBuffer( m_bufferId, frame.Descriptor() );

    QCStatus_e ret = m_node->ProcessFrameDescriptor( frameDesc );
    if ( QC_STATUS_OK != ret )
    {
        m_errorCount++;
        if ( m_errorCount <= kMaxReportedErrors )
        {
            QC_ERROR( "ProcessFrameDescriptor failed: %d", ret );
        }
    }
    else if ( false == m_isHcmToRos )
    {
        // A fetch that found no ROS message returns OK and counts in noDataCount.
        qrb::adapter::HcmIpcAdapterMonitoringData monitor{};
        if ( GetMonitor( monitor ) && ( monitor.noDataCount != m_noDataCount ) )
        {
            m_noDataCount = monitor.noDataCount;
            ret = QC_STATUS_TIMEOUT;
        }
    }
    if ( QC_STATUS_OK == ret )
    {
        m_frames++;
        m_rate.Add();
    }

    return ret;
}

void SampleIpcAdapter::SendFromTopic()
{
    while ( false == m_stop )
    {
        ContractFrameRef_t frame;
        if ( QC_STATUS_OK == m_sub.Receive( frame, kReceiveTimeoutMs ) )
        {
            (void) Process( *frame );
            // Hand the frame back to its pool.
            frame.reset();
        }
    }
}

void SampleIpcAdapter::FillFromTopic()
{
    ContractFrameRef_t frame;

    while ( false == m_stop )
    {
        if ( ( nullptr == frame ) && ( QC_STATUS_OK != m_sub.Receive( frame, kReceiveTimeoutMs ) ) )
        {
            continue;
        }

        QCStatus_e ret = Process( *frame );
        if ( QC_STATUS_OK == ret )
        {
            (void) m_pub.Publish( frame );
        }
        // Keep the frame only while no message waits; hand a filled or failed one back.
        if ( QC_STATUS_TIMEOUT != ret )
        {
            frame.reset();
        }
        if ( QC_STATUS_OK != ret )
        {
            std::this_thread::sleep_for( kNoDataWait );
        }
    }
}

void SampleIpcAdapter::ThreadMain()
{
    qrb::adapter::HcmIpcAdapterMonitoringData monitor{};
    if ( GetMonitor( monitor ) )
    {
        m_noDataCount = monitor.noDataCount;
    }

    if ( m_isHcmToRos )
    {
        SendFromTopic();
    }
    else
    {
        FillFromTopic();
    }
}

void SampleIpcAdapter::LogMain()
{
    std::unique_lock<std::mutex> lock( m_logMutex );
    while ( !m_logWake.wait_for( lock, kLogPeriod, [this] { return m_stop.load(); } ) )
    {
        PrintMonitor( true );
    }
}

QCStatus_e SampleIpcAdapter::Stop()
{
    {
        std::lock_guard<std::mutex> lock( m_logMutex );
        m_stop = true;
    }
    m_logWake.notify_all();
    if ( m_logThread.joinable() )
    {
        m_logThread.join();
    }
    if ( m_thread.joinable() )
    {
        m_thread.join();
    }

    QCStatus_e ret = m_node->Stop();
    PrintState( "Stop", ret );
    if ( QC_STATUS_OK != ret )
    {
        QC_ERROR( "failed to stop the IPC Adapter node: %d", ret );
    }
    PrintMonitor( false );
    Print( Text( "%s %llu frames, %llu errors", m_isHcmToRos ? "sent" : "received",
                 (unsigned long long) m_frames, (unsigned long long) m_errorCount ) );
    if ( ( QC_STATUS_OK == ret ) && ( 0 != m_errorCount ) )
    {
        ret = QC_STATUS_FAIL;
    }

    return ret;
}

QCStatus_e SampleIpcAdapter::Deinit()
{
    QCStatus_e ret = QC_STATUS_OK;

    if ( nullptr != m_node )
    {
        ret = m_node->DeInitialize();
        PrintState( "DeInitialize", ret );
        m_node.reset();
    }
    if ( nullptr != m_log )
    {
        (void) fclose( m_log );
        m_log = nullptr;
    }

    return ret;
}

REGISTER_SAMPLE( IpcAdapter, SampleIpcAdapter );

}   // namespace sample
}   // namespace QC
