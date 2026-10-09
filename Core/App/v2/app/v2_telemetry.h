#ifndef V2_TELEMETRY_H
#define V2_TELEMETRY_H

/* v2 遥测（冻结@M0）
   上电横幅（FW_TAG + 编译时间）+ 10Hz CSV 行：
   $V2,FW,ms,IR,act,err,OD,YAW,GZ,ISRus,OL,CT */
void v2_telemetry_banner(void);
void v2_telemetry_csv(void);

#endif /* V2_TELEMETRY_H */
