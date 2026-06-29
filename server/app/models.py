# DB 테이블 정의 (SQLAlchemy ORM)
# 각 클래스가 PostgreSQL 테이블 하나에 대응
# ENUM 타입은 DB 레벨에서 잘못된 값을 원천 차단
import enum
from sqlalchemy import (
    BigInteger, Column, Enum, ForeignKey, Integer, String, TIMESTAMP
)
from sqlalchemy.orm import DeclarativeBase


class Base(DeclarativeBase):
    pass


# ── ENUM 타입 정의 ────────────────────────────────────────────────────────────

class StorageType(enum.Enum):
    chilled = "chilled"   # 냉장 (A창고)
    frozen  = "frozen"    # 냉동 (B창고)


class OrderStatus(enum.Enum):
    pending          = "pending"           # 접수
    processing       = "processing"        # 출고 중 (로봇 이동)
    awaiting_pickup  = "awaiting_pickup"   # 게이트 도착, 수령 대기
    delivered        = "delivered"         # 수령 완료
    cancelled        = "cancelled"         # 취소


class CancelReason(enum.Enum):
    user        = "user"         # 고객 직접 취소
    no_pickup   = "no_pickup"    # 미수령 타임아웃
    misdelivery = "misdelivery"  # 오배송 관리자 정정


class UserRole(enum.Enum):
    customer = "customer"
    admin    = "admin"


class TaskType(enum.Enum):
    inbound  = "inbound"   # 입고 (물건 → 슬롯)
    outbound = "outbound"  # 출고 (슬롯 → 게이트)
    reclaim  = "reclaim"   # 회수 (게이트 → 슬롯, 미수령/오배송)


class TaskStatus(enum.Enum):
    pending   = "pending"    # 생성됨, 로봇 미배정
    assigned  = "assigned"   # 로봇 배정됨
    done      = "done"       # 완료
    cancelled = "cancelled"  # 취소
    failed    = "failed"     # 실패 (타임아웃 등)


class TaskEvent(enum.Enum):
    created   = "created"
    assigned  = "assigned"
    timed_out = "timed_out"
    done      = "done"
    cancelled = "cancelled"
    failed    = "failed"


# ── 테이블 정의 ───────────────────────────────────────────────────────────────

class Product(Base):
    __tablename__ = "products"

    name         = Column(String, primary_key=True)
    storage_type = Column(Enum(StorageType), nullable=False)


class User(Base):
    __tablename__ = "users"

    id            = Column(Integer, primary_key=True, autoincrement=True)
    email         = Column(String, unique=True, nullable=False)
    password_hash = Column(String, nullable=False)  # pgcrypto bcrypt 해시
    name          = Column(String, nullable=False)
    role          = Column(Enum(UserRole), nullable=False, default=UserRole.customer)


class Task(Base):
    __tablename__ = "tasks"

    id                  = Column(Integer, primary_key=True, autoincrement=True)
    type                = Column(Enum(TaskType), nullable=False)
    status              = Column(Enum(TaskStatus), nullable=False, default=TaskStatus.pending)
    picked_at           = Column(TIMESTAMP(timezone=True), nullable=True)
    cancel_requested_at = Column(TIMESTAMP(timezone=True), nullable=True)
    assigned_at         = Column(TIMESTAMP(timezone=True), nullable=True)   # 장애복구 타임아웃 기준
    completed_at        = Column(TIMESTAMP(timezone=True), nullable=True)
    product_name        = Column(String, ForeignKey("products.name"), nullable=False)
    order_id            = Column(Integer, ForeignKey("orders.id"), nullable=True)
    robot_id            = Column(String, nullable=True)
    source_location_id  = Column(String, nullable=False)
    target_location_id  = Column(String, nullable=False)
    created_at          = Column(TIMESTAMP(timezone=True), nullable=False)


class Location(Base):
    __tablename__ = "locations"

    location_id  = Column(String, primary_key=True)             # ex) SLOT-A1
    storage_type = Column(Enum(StorageType), nullable=False)
    product_name = Column(String, ForeignKey("products.name"), nullable=True)
    inbound_at   = Column(TIMESTAMP(timezone=True), nullable=True)
    # 어떤 task가 이 슬롯을 예약 중인지 — NULL이면 여유 슬롯, 재고 이중 주문 방지에 사용
    reserved_by  = Column(Integer, ForeignKey("tasks.id"), nullable=True)


class Order(Base):
    __tablename__ = "orders"

    id            = Column(Integer, primary_key=True, autoincrement=True)
    user_id       = Column(Integer, ForeignKey("users.id"), nullable=False)
    product_name  = Column(String, ForeignKey("products.name"), nullable=False)
    status        = Column(Enum(OrderStatus), nullable=False)
    cancel_reason = Column(Enum(CancelReason), nullable=True)
    created_at    = Column(TIMESTAMP(timezone=True), nullable=False)


class EventLog(Base):
    __tablename__ = "event_logs"

    # 임무 상태 변화 이력 — 장애 추적 및 면접 시연용
    id          = Column(BigInteger, primary_key=True, autoincrement=True)
    task_id     = Column(Integer, ForeignKey("tasks.id"), nullable=False)
    event       = Column(Enum(TaskEvent), nullable=False)
    robot_id    = Column(String, nullable=True)
    occurred_at = Column(TIMESTAMP(timezone=True), nullable=False)
