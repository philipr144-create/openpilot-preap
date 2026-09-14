# Optional lane centering and Tesla chimes

## Lane Centering Assist (Experimental)

Settings → NAP offers an OFF-by-default Lane Centering Assist toggle and Low / Medium / High strength (default Low). It applies bounded position and heading feedback relative to the model trajectory before the existing curvature and actuator limits. It does not alter the displayed green model path.

The feature requires active lateral control, calibrated and healthy inputs, strong lane probabilities, plausible and consistent lane geometry, and ordinary cruising from 20–70 mph. It yields for driver steering, signals, lane-change/turn desire, low-speed-turn smoothing and sharp curves. Additional lateral acceleration is capped at 0.15 m/s²; normal application/fade jerk at 0.10 m/s³. Disabling or a hard eligibility failure resets the added correction before stock limits. There is no integral accumulation. It cannot guarantee physical lane centering.

Validation includes focused logic tests, recorded-input replay, native cereal replay, and idealized straight-road simulations with delay. This remains unvalidated on the road. Turn the toggle OFF to restore original model curvature behavior. Change strength while parked.

## Tesla chimes

The comma 3X (`tizi`) engagement and disengagement WAV assets, plus the refusal asset, use converted recordings published in Tesla's owner manual:

- Engage: https://service.tesla.com/docs/Public/om_media/autosteer_enabled.mp3
- Disengage: https://service.tesla.com/docs/Public/om_media/autosteer_disabled.mp3
- Refuse: https://service.tesla.com/docs/Public/om_media/autopilot_unavailable.mp3
- Source guide: https://www.tesla.com/ownersmanual/models/en_us/GUID-AA58ED67-9C93-4EE6-8B19-9FDABE018787.html

Recordings are from Tesla; no authorship or license ownership is claimed. Converted to mono 48 kHz signed 16-bit PCM WAV, scaled near the existing chimes' active RMS levels without clipping, and padded with silence to complete 4096-frame output blocks. Urgent and driver-attention alerts remain unchanged. No soundd implementation changes.

Verification: all eight configured alerts load; replacement chimes render through the existing Soundd code and terminate correctly; a silent hardware output stream starts at 48 kHz, mono. The prior reported soundd failure could not be reproduced or diagnosed historically. Incompatible channels, sample rate or WAV encoding would fail this loader's assertions.
