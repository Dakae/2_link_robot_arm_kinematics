import tkinter as tk
from tkinter import ttk
from tkinter import font as tkFont
import math

# --- 기본 설정 및 상수 ---
INITIAL_VISUAL_SCALE = 0.5 # 화면에 그릴 때 적용할 초기/최대 스케일
INITIAL_WIDTH = 900
INITIAL_HEIGHT = 600

class RobotArmSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("2-Link Robot Arm Kinematics v7.2")
        self.root.geometry(f"{INITIAL_WIDTH}x{INITIAL_HEIGHT}")

        self.visual_scale = INITIAL_VISUAL_SCALE # 현재 적용되는 시각적 스케일

        # --- 스타일 및 폰트 설정 ---
        self.style = ttk.Style(root)
        self.style.theme_use('clam')
        default_font = tkFont.nametofont("TkDefaultFont")
        default_font.configure(size=10)
        
        # --- Tkinter 변수 설정 ---
        self.l1_var = tk.DoubleVar(value=400.0)
        self.l2_var = tk.DoubleVar(value=300.0)
        self.theta1_var = tk.DoubleVar(value=45.0)
        self.theta2_var = tk.DoubleVar(value=-60.0)
        self.x_var = tk.DoubleVar()
        self.y_var = tk.DoubleVar()
        
        self.current_t1_rad = 0.0
        self.current_t2_rad = 0.0

        # --- 메인 프레임 설정 ---
        main_frame = ttk.Frame(root, padding="10")
        main_frame.pack(expand=True, fill=tk.BOTH)

        control_frame = ttk.Frame(main_frame, width=280)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        control_frame.pack_propagate(False)

        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH)
        self.canvas = tk.Canvas(canvas_frame, bg='#FFFFFF', highlightthickness=1, highlightbackground='#CCCCCC')
        self.canvas.pack(expand=True, fill=tk.BOTH)

        self.setup_controls(control_frame)

        # --- 이벤트 바인딩 ---
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_click)
        self.canvas.bind("<Configure>", self.on_resize)

        self.root.after(100, self.initial_setup)

    def initial_setup(self):
        """UI가 완전히 그려진 후 초기 설정을 실행합니다."""
        self.update_visual_scale()
        self.update_from_angles()

    def setup_controls(self, parent):
        """컨트롤 패널의 위젯들을 생성하고 배치합니다."""
        
        # 링크 설정
        link_frame = ttk.LabelFrame(parent, text="링크 설정 (Link Configuration)", padding="10")
        link_frame.pack(fill="x", pady=(0, 10))
        link_frame.columnconfigure(1, weight=1)

        ttk.Label(link_frame, text="Link 1 Length:").grid(row=0, column=0, sticky="w", pady=2)
        e_l1 = ttk.Entry(link_frame, textvariable=self.l1_var)
        e_l1.grid(row=0, column=1, padx=5, sticky="ew")

        ttk.Label(link_frame, text="Link 2 Length:").grid(row=1, column=0, sticky="w", pady=2)
        e_l2 = ttk.Entry(link_frame, textvariable=self.l2_var)
        e_l2.grid(row=1, column=1, padx=5, sticky="ew")

        update_link_button = ttk.Button(link_frame, text="링크 길이 적용", command=self.apply_link_lengths)
        update_link_button.grid(row=2, columnspan=2, pady=(10, 0), sticky="ew")

        e_l1.bind("<Return>", lambda e: self.apply_link_lengths())
        e_l2.bind("<Return>", lambda e: self.apply_link_lengths())

        # 정기구학
        fk_frame = ttk.LabelFrame(parent, text="정기구학 (Forward Kinematics)", padding="10")
        fk_frame.pack(fill="x", pady=(0, 10))
        fk_frame.columnconfigure(1, weight=1)
        
        ttk.Label(fk_frame, text="Joint 1 (θ1, deg):").grid(row=0, column=0, sticky="w", pady=2)
        e1 = ttk.Entry(fk_frame, textvariable=self.theta1_var)
        e1.grid(row=0, column=1, padx=5, sticky="ew")
        s1 = ttk.Scale(fk_frame, from_=-180, to=180, orient='horizontal', variable=self.theta1_var, command=lambda v: self.update_from_angles())
        s1.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(0, 10))

        ttk.Label(fk_frame, text="Joint 2 (θ2, deg):").grid(row=2, column=0, sticky="w", pady=2)
        e2 = ttk.Entry(fk_frame, textvariable=self.theta2_var)
        e2.grid(row=2, column=1, padx=5, sticky="ew")
        s2 = ttk.Scale(fk_frame, from_=-180, to=180, orient='horizontal', variable=self.theta2_var, command=lambda v: self.update_from_angles())
        s2.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(0, 5))

        update_fk_button = ttk.Button(fk_frame, text="각도 적용", command=self.update_from_angles)
        update_fk_button.grid(row=4, columnspan=2, pady=(10, 0), sticky="ew")
        
        # 역기구학
        ik_frame = ttk.LabelFrame(parent, text="역기구학 (Inverse Kinematics)", padding="10")
        ik_frame.pack(fill="x", pady=10)
        ik_frame.columnconfigure(1, weight=1)

        max_reach = self.l1_var.get() + self.l2_var.get()

        ttk.Label(ik_frame, text="End-effector X:").grid(row=0, column=0, sticky="w", pady=2)
        e3 = ttk.Entry(ik_frame, textvariable=self.x_var)
        e3.grid(row=0, column=1, sticky="ew", padx=5)
        self.x_scale = ttk.Scale(ik_frame, from_=-max_reach, to=max_reach, orient='horizontal', variable=self.x_var, command=lambda v: self.update_from_coords())
        self.x_scale.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(0, 10))
        
        ttk.Label(ik_frame, text="End-effector Y:").grid(row=2, column=0, sticky="w", pady=2)
        e4 = ttk.Entry(ik_frame, textvariable=self.y_var)
        e4.grid(row=2, column=1, sticky="ew", padx=5)
        self.y_scale = ttk.Scale(ik_frame, from_=-max_reach, to=max_reach, orient='horizontal', variable=self.y_var, command=lambda v: self.update_from_coords())
        self.y_scale.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(0, 5))

        update_ik_button = ttk.Button(ik_frame, text="좌표 적용", command=self.update_from_coords)
        update_ik_button.grid(row=4, columnspan=2, pady=(10, 0), sticky="ew")
        
        self.status_label = ttk.Label(parent, text="상태: 준비 완료", foreground="blue", anchor="center")
        self.status_label.pack(pady=10, fill="x")
        
        e1.bind("<Return>", lambda e: self.update_from_angles())
        e2.bind("<Return>", lambda e: self.update_from_angles())
        e3.bind("<Return>", lambda e: self.update_from_coords())
        e4.bind("<Return>", lambda e: self.update_from_coords())

    def update_visual_scale(self):
        """캔버스 크기에 맞춰 로봇팔의 시각적 스케일을 자동으로 조절합니다."""
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        
        if width <= 1 or height <= 1: return

        L1 = self.l1_var.get()
        L2 = self.l2_var.get()
        max_reach = L1 + L2
        if max_reach == 0: return

        scale_x = (width * 0.95) / (max_reach * 2)
        scale_y = (height * 0.95) / (max_reach * 2)
        
        required_scale = min(scale_x, scale_y)
        self.visual_scale = min(INITIAL_VISUAL_SCALE, required_scale)

    def update_slider_ranges(self):
        """링크 길이에 따라 X, Y 좌표 슬라이더의 범위를 업데이트합니다."""
        max_reach = self.l1_var.get() + self.l2_var.get()
        self.x_scale.config(from_=-max_reach, to=max_reach)
        self.y_scale.config(from_=-max_reach, to=max_reach)

    def apply_link_lengths(self):
        """링크 길이가 변경되었을 때 시뮬레이션을 업데이트합니다."""
        self.update_slider_ranges()
        self.update_visual_scale()
        self.update_from_angles()

    def update_from_angles(self):
        L1 = self.l1_var.get()
        L2 = self.l2_var.get()

        # 슬라이더 값의 소수점을 정리하여 Entry 위젯에 반영
        self.theta1_var.set(round(self.theta1_var.get(), 2))
        self.theta2_var.set(round(self.theta2_var.get(), 2))
        
        t1_deg = self.theta1_var.get()
        t2_deg = self.theta2_var.get()

        self.current_t1_rad = math.radians(t1_deg)
        self.current_t2_rad = math.radians(t2_deg)
        
        x1 = L1 * math.cos(self.current_t1_rad)
        y1 = L1 * math.sin(self.current_t1_rad)
        x2 = x1 + L2 * math.cos(self.current_t1_rad + self.current_t2_rad)
        y2 = y1 + L2 * math.sin(self.current_t1_rad + self.current_t2_rad)

        self.x_var.set(round(x2, 2))
        self.y_var.set(round(y2, 2))
        
        self.draw_robot()
        self.status_label.config(text="상태: 각도 적용 완료", foreground="green")

    def update_from_coords(self):
        L1 = self.l1_var.get()
        L2 = self.l2_var.get()

        # 슬라이더 값의 소수점을 정리하여 Entry 위젯에 반영
        self.x_var.set(round(self.x_var.get(), 2))
        self.y_var.set(round(self.y_var.get(), 2))
        
        x = self.x_var.get()
        y = self.y_var.get()

        try:
            dist_sq = x**2 + y**2
            if not (L1 - L2)**2 <= dist_sq <= (L1 + L2)**2:
                raise ValueError("도달할 수 없는 좌표입니다.")

            cos_t2 = (dist_sq - L1**2 - L2**2) / (2 * L1 * L2)
            cos_t2 = max(-1.0, min(1.0, cos_t2))
            
            t2_rad = math.acos(cos_t2)
            k1 = L1 + L2 * math.cos(t2_rad)
            k2 = L2 * math.sin(t2_rad)
            t1_rad = math.atan2(y, x) - math.atan2(k2, k1)

            # 계산된 각도를 [-pi, pi] 범위로 변환합니다.
            t1_rad = (t1_rad + math.pi) % (2 * math.pi) - math.pi
            
            self.current_t1_rad, self.current_t2_rad = t1_rad, t2_rad
            
            self.theta1_var.set(round(math.degrees(t1_rad), 2))
            self.theta2_var.set(round(math.degrees(t2_rad), 2))

            self.draw_robot()
            self.status_label.config(text="상태: 좌표 적용 완료", foreground="green")

        except ValueError as e:
            self.status_label.config(text=f"오류: {e}", foreground="red")

    def draw_robot(self):
        L1 = self.l1_var.get()
        L2 = self.l2_var.get()
        self.canvas.delete("all")
        
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        origin_x = width / 2
        origin_y = height / 2

        self.canvas.create_line(0, origin_y, width, origin_y, fill='#CCCCCC', dash=(2, 2))
        self.canvas.create_line(origin_x, 0, origin_x, height, fill='#CCCCCC', dash=(2, 2))
        self.canvas.create_text(width - 20, origin_y - 10, text="+X", fill='gray', font=('Arial', 9))
        self.canvas.create_text(20, origin_y - 10, text="-X", fill='gray', font=('Arial', 9))
        self.canvas.create_text(origin_x + 20, 10, text="+Y", fill='gray', font=('Arial', 9))
        self.canvas.create_text(origin_x + 20, height - 10, text="-Y", fill='gray', font=('Arial', 9))
        
        scaled_L1 = L1 * self.visual_scale
        scaled_L2 = L2 * self.visual_scale
        
        j1_x = origin_x + scaled_L1 * math.cos(self.current_t1_rad)
        j1_y = origin_y - scaled_L1 * math.sin(self.current_t1_rad)
        ee_x = j1_x + scaled_L2 * math.cos(self.current_t1_rad + self.current_t2_rad)
        ee_y = j1_y - scaled_L2 * math.sin(self.current_t1_rad + self.current_t2_rad)
        
        max_r, min_r = (L1 + L2) * self.visual_scale, abs(L1 - L2) * self.visual_scale
        self.canvas.create_oval(origin_x - max_r, origin_y - max_r, origin_x + max_r, origin_y + max_r, outline='#E0E0E0', dash=(4, 4))
        if min_r > 0: self.canvas.create_oval(origin_x - min_r, origin_y - min_r, origin_x + min_r, origin_y + min_r, outline='#E0E0E0', dash=(4, 4))

        self.canvas.create_line(origin_x, origin_y, j1_x, j1_y, fill='#00529B', width=6, capstyle='round')
        self.canvas.create_line(j1_x, j1_y, ee_x, ee_y, fill='#D81B60', width=6, capstyle='round')
        
        self.canvas.create_oval(origin_x-6, origin_y-6, origin_x+6, origin_y+6, fill='black', outline='white', width=2)
        self.canvas.create_oval(j1_x-6, j1_y-6, j1_x+6, j1_y+6, fill='#424242', outline='white', width=2)
        self.canvas.create_oval(ee_x-6, ee_y-6, ee_x+6, ee_y+6, fill='#8E24AA', outline='white', width=2)

    def on_canvas_click(self, event):
        origin_x = self.canvas.winfo_width() / 2
        origin_y = self.canvas.winfo_height() / 2
        
        if self.visual_scale < 1e-6: return

        target_x = (event.x - origin_x) / self.visual_scale
        target_y = (origin_y - event.y) / self.visual_scale
        
        self.x_var.set(round(target_x, 2))
        self.y_var.set(round(target_y, 2))
        self.update_from_coords()

    def on_resize(self, event):
        """창 크기가 변경될 때마다 스케일을 재계산하고 로봇팔을 다시 그립니다."""
        self.update_visual_scale()
        self.draw_robot()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmSimulator(root)
    root.mainloop()

