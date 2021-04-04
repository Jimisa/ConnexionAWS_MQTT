#include <pgmspace.h>

#define SECRET
#define THINGNAME ""

const char WIFI_SSID[] = "Livebox-13B8"; //"AndroidAP";
const char WIFI_PASSWORD[] = "F97E2A913FC79E5D3C49A71C7E";;//"eba9d278808f";
const char AWS_IOT_ENDPOINT[] = "xxxxx.amazonaws.com";

const char BROKER_ADDR[] = "192.168.1.27"; //14"; 
const int BROKER_PORT = 1883;
const char BROKER_USER[] = "homeadmin";
const char BROKER_PWD[] = "AdminMQTT&28";
 
// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
-----END RSA PRIVATE KEY-----
)KEY";