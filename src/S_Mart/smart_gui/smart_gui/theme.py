"""목업(docs_hub/mockup/admin-ui-v2.html) 팔레트 → Qt 스타일시트.

색상 이름은 목업 CSS 변수명을 그대로 따른다 (--panel → PANEL 등).
목업을 고칠 일이 생기면 여기 값만 맞추면 된다.
"""

BG = '#0b1220'
PANEL = '#131c2e'
PANEL2 = '#0f1726'
LINE = '#243049'
INK = '#e6edf6'
MUTED = '#8a99b3'
DIM = '#5d6b86'

CYAN = '#22d3ee'
CYAN_D = '#0e7490'
ORANGE = '#fb923c'
GREEN = '#34d399'
AMBER = '#fbbf24'
RED = '#f87171'
BLUE = '#60a5fa'
VIOLET = '#a78bfa'

# 로봇별 고정 색 (맵 점·아바타·범례가 전부 이걸 참조)
ROBOT_COLORS = {'AMR_1': CYAN, 'AMR_2': ORANGE}

MONO = 'monospace'

QSS = f"""
QWidget {{
    background: {BG};
    color: {INK};
    font-family: "Pretendard", "Noto Sans CJK KR", sans-serif;
    font-size: 13px;
}}
QToolTip {{
    background: {PANEL2};
    color: {INK};
    border: 1px solid {LINE};
    padding: 5px;
}}

/* ── 패널 (목업 .panel) ── */
QFrame#Panel {{
    background: {PANEL};
    border: 1px solid {LINE};
    border-radius: 12px;
}}
QLabel#PanelHead {{
    color: #ffffff;
    font-weight: 700;
    background: transparent;
}}
QLabel#PanelHeadCnt {{
    color: {MUTED};
    font-size: 11px;
    font-weight: 600;
    background: transparent;
}}

/* ── 상단바 ── */
QFrame#TopBar {{
    background: {PANEL};
    border-bottom: 1px solid {LINE};
}}
QLabel#Brand {{
    color: #ffffff;
    font-size: 18px;
    font-weight: 800;
    background: transparent;
}}
QLabel#BrandTag {{
    color: {CYAN};
    border: 1px solid {CYAN_D};
    border-radius: 9px;
    padding: 2px 8px;
    font-size: 11px;
    font-weight: 600;
    background: transparent;
}}
QLabel#Pill {{
    background: {PANEL2};
    border: 1px solid {LINE};
    border-radius: 8px;
    padding: 6px 12px;
    color: {MUTED};
}}
QLabel#Clock {{
    color: {MUTED};
    font-family: {MONO};
    background: transparent;
}}
/* ── 사이드바 ── */
QFrame#Nav {{
    background: {PANEL};
    border-right: 1px solid {LINE};
}}
QLabel#NavSec {{
    color: {DIM};
    font-size: 10px;
    font-weight: 700;
    padding: 12px 12px 4px;
    background: transparent;
}}
QPushButton#NavItem {{
    text-align: left;
    padding: 10px 12px;
    border: none;
    border-radius: 9px;
    color: {MUTED};
    font-weight: 600;
    background: transparent;
}}
QPushButton#NavItem:hover {{
    background: {PANEL2};
    color: {INK};
}}
QPushButton#NavItem:checked {{
    background: rgba(34, 211, 238, 0.12);
    color: {CYAN};
}}
QPushButton#NavItem:disabled {{
    color: {DIM};
}}
QLabel#NavFoot {{
    color: {DIM};
    font-size: 11px;
    padding: 10px 12px;
    background: transparent;
}}

/* ── 페이지 제목 ── */
QLabel#PageTitle {{
    color: #ffffff;
    font-size: 16px;
    font-weight: 800;
    background: transparent;
}}
QLabel#PageSub {{
    color: {MUTED};
    font-size: 12px;
    background: transparent;
}}

/* ── KPI 타일 ── */
QLabel#KpiLbl {{
    color: {MUTED};
    font-size: 11px;
    font-weight: 600;
    background: transparent;
}}
QLabel#KpiVal {{
    color: #ffffff;
    font-size: 26px;
    font-weight: 800;
    background: transparent;
}}
QLabel#KpiSub {{
    color: {MUTED};
    font-size: 11px;
    font-weight: 700;
    background: transparent;
}}

/* ── 로봇 카드 ── */
QLabel#RName {{
    color: #ffffff;
    font-weight: 700;
    background: transparent;
}}
QLabel#RSub {{
    color: {MUTED};
    font-size: 11px;
    background: transparent;
}}
QLabel#RMeta {{
    color: {MUTED};
    font-size: 12px;
    background: transparent;
}}
QPushButton#RBtn {{
    background: {PANEL2};
    border: 1px solid {LINE};
    color: {MUTED};
    padding: 7px;
    border-radius: 7px;
    font-size: 12px;
    font-weight: 600;
}}
QPushButton#RBtn:hover {{
    color: {INK};
    border-color: {DIM};
}}
QPushButton#RBtn:disabled {{
    color: {DIM};
    border-color: {LINE};
    background: transparent;
}}

/* ── 리스트 행 ── */
QLabel#RowMain {{
    color: #ffffff;
    font-weight: 600;
    background: transparent;
}}
QLabel#RowMeta {{
    color: {MUTED};
    font-size: 11px;
    background: transparent;
}}
QLabel#Empty {{
    color: {DIM};
    font-size: 12px;
    background: transparent;
}}

/* ── 스크롤바 ── */
QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{
    background: transparent;
    width: 8px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: {LINE};
    border-radius: 4px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{ background: {DIM}; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: transparent; }}
"""


def chip_qss(fg: str, alpha: float = 0.15) -> str:
    """상태 뱃지(목업 .sbadge/.rstat) — 전경색에서 배경을 파생시킨다."""
    r = int(fg[1:3], 16)
    g = int(fg[3:5], 16)
    b = int(fg[5:7], 16)
    return (
        f'background: rgba({r},{g},{b},{alpha});'
        f'color: {fg};'
        'border-radius: 9px;'
        'padding: 3px 9px;'
        'font-size: 10px;'
        'font-weight: 700;'
    )
