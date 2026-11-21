uint32 candidate_id
uint32[] segment_ids
float32 observed_length   # [m]
float32 heading_change    # [rad]
float32 curvature         # [1/m]
float32 rule_score
float32 mlp_score         # relative ranking score, NaN when MLP unavailable (not a probability)
bool geometry_feasible
bool selected
string reason
