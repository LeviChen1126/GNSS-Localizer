/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gnss/geo_pos_conv.hpp>

geo_pos_conv::geo_pos_conv()
    : m_x(0)
    , m_y(0)
    , m_z(0)
    , m_lat(0)
    , m_lon(0)
    , m_h(0)
    , m_PLato(0)
    , m_PLo(0)
{
}

double geo_pos_conv::x() const
{
  return m_x;
}

double geo_pos_conv::y() const
{
  return m_y;
}

double geo_pos_conv::z() const
{
  return m_z;
}

void geo_pos_conv::set_plane(double lat, double lon)
{
  m_PLato = lat;
  m_PLo = lon;
}

void geo_pos_conv::set_plane(int num)
{
  double lon_deg, lon_min, lat_deg, lat_min; // longitude and latitude of origin of each plane in Japan

  //寧馨公園經緯度座標
  //經度: 121.5362456211845
  //緯度: 25.043654326395625

  //以下座標點（1~18）為臺灣衛星追蹤站座標,其中 Plane 13 是北科旁寧馨公園一等水準點
  //成功
  if (num == 1)
  {
    lat_deg = 23;
    lat_min = 5.899398806799994;
    lon_deg = 121;
    lon_min = 22.468138545951035;
  }

  //成功大學
  else if (num == 2)
  {
    lat_deg = 22;
    lat_min = 59.92639900196458;
    lon_deg = 120;
    lon_min = 13.195336390761838;
  }

  //鳳林
  else if (num == 3)
  {
    lat_deg = 23;
    lat_min = 44.77764802060449;
    lon_deg = 121;
    lon_min = 27.198142096524975;
  }

  //竹南
  else if (num == 4)
  {
    lat_deg = 24;
    lat_min = 41.03279039150543;
    lon_deg = 120;
    lon_min = 52.51754843977352;
  }

  //高雄港
  else if (num == 5)
  {
    lat_deg = 22;
    lat_min = 36.869280098031325;
    lon_deg = 120;
    lon_min = 17.30084050954872;
  }

  //墾丁
  else if (num == 6)
  {
    lat_deg = 21;
    lat_min = 56.9635992265826;
    lon_deg = 120;
    lon_min = 46.91820753956648;
  }

  //金門
  else if (num == 7)
  {
    lat_deg = 24;
    lat_min = 27.82463988952763;
    lon_deg = 120;
    lon_min = 23.310468095941985;
  }

  //馬祖
  else if (num == 8)
  {
    lat_deg = 26;
    lat_min = 9.42327390661994;
    lon_deg = 121;
    lon_min = 55.98691278105605;
  }

  //北港
  else if (num == 9)
  {
    lat_deg = 23;
    lat_min = 34.79324345098853;
    lon_deg = 120;
    lon_min = 18.328951071513018;
  }

  //埔里
  else if (num == 10)
  {
    lat_deg = 23;
    lat_min = 58.43251152146216;
    lon_deg = 120;
    lon_min = 58.91511703948254;
  }

  //台中港
  else if (num == 11)
  {
    lat_deg = 24;
    lat_min = 17.449213879309227;
    lon_deg = 120;
    lon_min = 32.10784029176921;
  }

  //太麻里
  else if (num == 12)
  {
    lat_deg = 22;
    lat_min = 36.96394342762858;
    lon_deg = 121;
    lon_min = 0.443580139650237;
  }

  //北科旁寧馨公園
  //經度: 121.5362456211845
  //緯度: 25.043654326395625
  else if (num == 13)
  {
    lat_deg = 25;
    lat_min = 2.6192595837375166;
    lon_deg = 121;
    lon_min = 32.17473727107034;
  }

  //外垵
  else if (num == 14)
  {
    lat_deg = 23;
    lat_min = 34.050721860633715;
    lon_deg = 121;
    lon_min = 28.8480924171904;
  }

  //武陵
  else if (num == 15)
  {
    lat_deg = 24;
    lat_min = 21.135309812715946;
    lon_deg = 121;
    lon_min = 18.503283665548338;
  }

  //霧鹿
  else if (num == 16)
  {
    lat_deg = 23;
    lat_min = 10.152032164347986;
    lon_deg = 121;
    lon_min = 2.4847614809729635;
  }

  //宜蘭
  else if (num == 17)
  {
    lat_deg = 24;
    lat_min = 44.760012057015146;
    lon_deg = 121;
    lon_min = 44.73823193353695;
  }

  //陽明山
  else if (num == 18)
  {
    lat_deg = 25;
    lat_min = 9.939701677178832;
    lon_deg = 121;
    lon_min = 34.43866041998774;
  }

  //Autoware 原生
  else if (num == 19)
  {
    lat_deg = 26;
    lat_min = 0;
    lon_deg = 154;
    lon_min = 0;
  }

  m_PLato = lat_deg + (lat_min / 60.0);
  m_PLo = lon_deg + (lon_min / 60.0);

  // // swap longitude and latitude
  // m_PLato = M_PI * ((double)lat_deg + (double)lat_min / 60.0) / 180.0;
  // m_PLo = M_PI * ((double)lon_deg + (double)lon_min / 60.0) / 180;
}

void geo_pos_conv::set_xyz(double cx, double cy, double cz)
{
  m_x = cx;
  m_y = cy;
  m_z = cz;
  conv_xyz2llh();
}

void geo_pos_conv::set_llh_nmea_degrees(double latd, double lond, double h)
{
  double lat, lad, lod, lon;
  // 1234.56 -> 12'34.56 -> 12+ 34.56/60

  lad = floor(latd / 100.);
  lat = latd - lad * 100.;
  lod = floor(lond / 100.);
  lon = lond - lod * 100.;

  m_lat = (lad + lat / 60.0);
  m_lon = (lod + lon / 60.0);
  m_h = h;

  // // Changing Longitude and Latitude to Radians
  // m_lat = (lad + lat / 60.0) * M_PI / 180;
  // m_lon = (lod + lon / 60.0) * M_PI / 180;
  // m_h = h;

  conv_llh2xyz();
}

void geo_pos_conv::llh_to_xyz(double lat, double lon, double ele)
{
  m_lat = lat * M_PI / 180;
  m_lon = lon * M_PI / 180;
  m_h = ele;

  conv_llh2xyz();
}

void geo_pos_conv::conv_llh2xyz(void)
{
  double PS;   //
  double PSo;  //
  double PDL;  //
  double Pt;   //
  double PN;   //
  double PW;   //

  double PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9;
  double PA, PB, PC, PD, PE, PF, PG, PH, PI;
  double Pe;   //
  double Pet;  //
  double Pnn;  //
  double AW, FW, Pmo;

  Pmo = 0.9999;

  /*WGS84 Parameters*/
  AW = 6378137.0;            // Semimajor Axis
  FW = 1.0 / 298.257222101;  // 298.257223563 //Geometrical flattening

  Pe = (double)sqrt(2.0 * FW - pow(FW, 2));
  Pet = (double)sqrt(pow(Pe, 2) / (1.0 - pow(Pe, 2)));

  PA = (double)1.0 + 3.0 / 4.0 * pow(Pe, 2) + 45.0 / 64.0 * pow(Pe, 4) + 175.0 / 256.0 * pow(Pe, 6) +
       11025.0 / 16384.0 * pow(Pe, 8) + 43659.0 / 65536.0 * pow(Pe, 10) + 693693.0 / 1048576.0 * pow(Pe, 12) +
       19324305.0 / 29360128.0 * pow(Pe, 14) + 4927697775.0 / 7516192768.0 * pow(Pe, 16);

  PB = (double)3.0 / 4.0 * pow(Pe, 2) + 15.0 / 16.0 * pow(Pe, 4) + 525.0 / 512.0 * pow(Pe, 6) +
       2205.0 / 2048.0 * pow(Pe, 8) + 72765.0 / 65536.0 * pow(Pe, 10) + 297297.0 / 262144.0 * pow(Pe, 12) +
       135270135.0 / 117440512.0 * pow(Pe, 14) + 547521975.0 / 469762048.0 * pow(Pe, 16);

  PC = (double)15.0 / 64.0 * pow(Pe, 4) + 105.0 / 256.0 * pow(Pe, 6) + 2205.0 / 4096.0 * pow(Pe, 8) +
       10395.0 / 16384.0 * pow(Pe, 10) + 1486485.0 / 2097152.0 * pow(Pe, 12) + 45090045.0 / 58720256.0 * pow(Pe, 14) +
       766530765.0 / 939524096.0 * pow(Pe, 16);

  PD = (double)35.0 / 512.0 * pow(Pe, 6) + 315.0 / 2048.0 * pow(Pe, 8) + 31185.0 / 131072.0 * pow(Pe, 10) +
       165165.0 / 524288.0 * pow(Pe, 12) + 45090045.0 / 117440512.0 * pow(Pe, 14) +
       209053845.0 / 469762048.0 * pow(Pe, 16);

  PE = (double)315.0 / 16384.0 * pow(Pe, 8) + 3465.0 / 65536.0 * pow(Pe, 10) + 99099.0 / 1048576.0 * pow(Pe, 12) +
       4099095.0 / 29360128.0 * pow(Pe, 14) + 348423075.0 / 1879048192.0 * pow(Pe, 16);

  PF = (double)693.0 / 131072.0 * pow(Pe, 10) + 9009.0 / 524288.0 * pow(Pe, 12) +
       4099095.0 / 117440512.0 * pow(Pe, 14) + 26801775.0 / 469762048.0 * pow(Pe, 16);

  PG = (double)3003.0 / 2097152.0 * pow(Pe, 12) + 315315.0 / 58720256.0 * pow(Pe, 14) +
       11486475.0 / 939524096.0 * pow(Pe, 16);

  PH = (double)45045.0 / 117440512.0 * pow(Pe, 14) + 765765.0 / 469762048.0 * pow(Pe, 16);

  PI = (double)765765.0 / 7516192768.0 * pow(Pe, 16);

  PB1 = (double)AW * (1.0 - pow(Pe, 2)) * PA;
  PB2 = (double)AW * (1.0 - pow(Pe, 2)) * PB / -2.0;
  PB3 = (double)AW * (1.0 - pow(Pe, 2)) * PC / 4.0;
  PB4 = (double)AW * (1.0 - pow(Pe, 2)) * PD / -6.0;
  PB5 = (double)AW * (1.0 - pow(Pe, 2)) * PE / 8.0;
  PB6 = (double)AW * (1.0 - pow(Pe, 2)) * PF / -10.0;
  PB7 = (double)AW * (1.0 - pow(Pe, 2)) * PG / 12.0;
  PB8 = (double)AW * (1.0 - pow(Pe, 2)) * PH / -14.0;
  PB9 = (double)AW * (1.0 - pow(Pe, 2)) * PI / 16.0;

  PS = (double)PB1 * m_lat + PB2 * sin(2.0 * m_lat) + PB3 * sin(4.0 * m_lat) + PB4 * sin(6.0 * m_lat) +
       PB5 * sin(8.0 * m_lat) + PB6 * sin(10.0 * m_lat) + PB7 * sin(12.0 * m_lat) + PB8 * sin(14.0 * m_lat) +
       PB9 * sin(16.0 * m_lat);

  PSo = (double)PB1 * m_PLato + PB2 * sin(2.0 * m_PLato) + PB3 * sin(4.0 * m_PLato) + PB4 * sin(6.0 * m_PLato) +
        PB5 * sin(8.0 * m_PLato) + PB6 * sin(10.0 * m_PLato) + PB7 * sin(12.0 * m_PLato) + PB8 * sin(14.0 * m_PLato) +
        PB9 * sin(16.0 * m_PLato);

  PDL = (double)m_lon - m_PLo;
  Pt = (double)tan(m_lat);
  PW = (double)sqrt(1.0 - pow(Pe, 2) * pow(sin(m_lat), 2));
  PN = (double)AW / PW;
  Pnn = (double)sqrt(pow(Pet, 2) * pow(cos(m_lat), 2));

  // m_x = (double)((PS - PSo) + (1.0 / 2.0) * PN * pow(cos(m_lat), 2.0) * Pt * pow(PDL, 2.0) +
  //                (1.0 / 24.0) * PN * pow(cos(m_lat), 4) * Pt *
  //                    (5.0 - pow(Pt, 2) + 9.0 * pow(Pnn, 2) + 4.0 * pow(Pnn, 4)) * pow(PDL, 4) -
  //                (1.0 / 720.0) * PN * pow(cos(m_lat), 6) * Pt *
  //                    (-61.0 + 58.0 * pow(Pt, 2) - pow(Pt, 4) - 270.0 * pow(Pnn, 2) + 330.0 * pow(Pt, 2) * pow(Pnn, 2)) *
  //                    pow(PDL, 6) -
  //                (1.0 / 40320.0) * PN * pow(cos(m_lat), 8) * Pt *
  //                    (-1385.0 + 3111 * pow(Pt, 2) - 543 * pow(Pt, 4) + pow(Pt, 6)) * pow(PDL, 8)) *
  //       Pmo;

  // m_y = (double)(PN * cos(m_lat) * PDL -
  //                1.0 / 6.0 * PN * pow(cos(m_lat), 3) * (-1 + pow(Pt, 2) - pow(Pnn, 2)) * pow(PDL, 3) -
  //                1.0 / 120.0 * PN * pow(cos(m_lat), 5) *
  //                    (-5.0 + 18.0 * pow(Pt, 2) - pow(Pt, 4) - 14.0 * pow(Pnn, 2) + 58.0 * pow(Pt, 2) * pow(Pnn, 2)) *
  //                    pow(PDL, 5) -
  //                1.0 / 5040.0 * PN * pow(cos(m_lat), 7) *
  //                    (-61.0 + 479.0 * pow(Pt, 2) - 179.0 * pow(Pt, 4) + pow(Pt, 6)) * pow(PDL, 7)) *
  //       Pmo;

  // m_z = m_h;

  m_x = (m_lat - m_PLato) * 110773.52297985487;
  m_y = (m_lon - m_PLo) * 100914.35933914837;

  m_z = 0;
}

void geo_pos_conv::conv_xyz2llh(void)
{
  // n/a
}
