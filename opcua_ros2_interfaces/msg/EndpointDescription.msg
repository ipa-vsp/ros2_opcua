uint8 SECURITY_MODE_NONE              = 1
uint8 SECURITY_MODE_SIGN              = 2
uint8 SECURITY_MODE_SIGN_AND_ENCRYPT  = 3

string endpoint_url
ApplicationDescription server
uint8[] server_certificate
uint8 security_mode
string security_policy_uri
# UserIdentityToken types supported by this endpoint.
# Each element is the policy URI string, e.g.
# "http://opcfoundation.org/UA/SecurityPolicy#None"
string[] user_identity_token_types
string transport_profile_uri
uint8 security_level
