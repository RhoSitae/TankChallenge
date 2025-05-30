import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

class BallisticCalculator:
    """
    사거리(range) ↔ 고저각(elevation angle) 변환을 위한 디지털 사표 역보간 클래스
    """
    def __init__(self, csv_path, kind='linear', extrapolate=True):
        """
        Parameters
        ----------
        csv_path : str
            'elevation_deg','range_m' 컬럼이 포함된 CSV 파일 경로
        kind : str, optional
            보간 방식 ('linear','quadratic','cubic' 등), 기본 'linear'
        extrapolate : bool, optional
            테이블 범위를 벗어나는 값에 대해 외삽 허용 여부, 기본 True
        """
        df = pd.read_csv(csv_path)
        angles = df['elevation_deg'].values
        ranges = df['range_m'].values
        
        # 사거리 기준으로 오름차순 정렬
        idx = np.argsort(ranges)
        self.ranges_sorted = ranges[idx]
        self.angles_sorted = angles[idx]
        
        # 역보간 함수 설정
        fill = 'extrapolate' if extrapolate else (np.nan, np.nan)
        self.inverse_interp = interp1d(
            self.ranges_sorted,
            self.angles_sorted,
            kind=kind,
            bounds_error=False,
            fill_value=fill
        )

    def get_elevation_for_range(self, target_range):
        """
        주어진 사거리(target_range)에 대응하는 고저각(elevation angle)을 반환합니다.
        
        Returns
        -------
        float
            계산된 고저각 (degree)
        """
        return float(self.inverse_interp(target_range))