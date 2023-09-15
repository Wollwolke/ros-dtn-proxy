#pragma once

namespace dtnproxy::common {

constexpr auto PACKAGE_NAME = "dtn_proxy";
constexpr auto DEFAULT_NODE_NAME = "dtnproxy";

constexpr auto REMOTE_CONFIG_ENDPOINT = "remoteConfig";
constexpr auto REMOTE_CONFIG_LIFETIME = 24 * 60 * 60;

namespace dtnPrefixes {
constexpr auto TOPIC = "rt_";
constexpr auto RESPONSE = "rr_";
constexpr auto REQUEST = "rq_";
constexpr auto INTERNAL = "dtnproxy_";
}  // namespace dtnPrefixes

}  // namespace dtnproxy::common
