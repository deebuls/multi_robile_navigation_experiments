# Video editing commands

## Fast forwarding and adding 2x
``` bash
$> ffmpeg -i takeover.mp4 -an -filter_complex "[0:v]setpts=0.5*PTS,scale=1280:-2,
drawtext=text='2x Speed':x=(w-text_w)/2:y=10:fontsize=30:fontcolor=white" -c:v libx264 -preset veryfast -crf 18 output.mp4
```



## Adding annotation to video
``` bash
ffmpeg -i output.mp4 -vf "drawtext=text='Robile 0 as LEADER':x=10:y=H-th-10:fontsize=50:fontcolor=yellow:enable='between(t,5,10)',
drawtext=text='Robile 1 as FOLLOWER':x=10:y=H-th-10:fontsize=50:fontcolor=yellow:enable='between(t,12,20)',
drawtext=text='EDDI msg highly_connected_platooning_3':x=10:y=H-th-10:fontsize=50:fontcolor=yellow:enable='between(t,22,40)', 
drawtext=text='Robile 0 navigation ERROR':x=10:y=H-th-10:fontsize=50:fontcolor=red:enable='between(t,43,48)', 
drawtext=text='EDDI msg follower_takeover':x=10:y=H-th-10:fontsize=50:fontcolor=yellow:enable='between(t,50,52)',
drawtext=text='Robile 1 STOP following':x=10:y=H-th-10:fontsize=50:fontcolor=red:enable='between(t,53,55)',
drawtext=text='Robile 1 TAKEOVER':x=10:y=H-th-10:fontsize=50:fontcolor=yellow:enable='between(t,56,70)' " -c:a copy anootate.mp4
```
