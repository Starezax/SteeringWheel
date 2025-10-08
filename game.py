import pygame
import serial
import re
import sys
import random

pygame.init()

WIDTH, HEIGHT = 800, 600
FPS = 60
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (100, 100, 100)
RED = (255, 50, 50)
GREEN = (50, 255, 50)
BLUE = (50, 150, 255)
YELLOW = (255, 255, 0)

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("STM32 Racing Game - Керування потенціометром")
clock = pygame.time.Clock()
font = pygame.font.Font(None, 36)
small_font = pygame.font.Font(None, 24)

def connect_serial():
    ports = ['COM6', 'COM7', 'COM8']
    for port in ports:
        try:
            ser = serial.Serial(port, 9600, timeout=0.1)
            print(f"Підключено до {port}")
            return ser
        except:
            continue
    print("Не вдалося підключитись до STM32!")
    print("Перевір COM порт в Device Manager")
    return None

class Car:
    def __init__(self):
        self.width = 40
        self.height = 80
        self.x = WIDTH // 2 - self.width // 2
        self.y = HEIGHT - 120
        self.color = RED
        
    def update(self, pot_value):
        road_left = 100
        road_width = WIDTH - 200
        self.x = road_left + int((pot_value / 4095.0) * road_width) - self.width // 2
        
    def draw(self, screen):
        pygame.draw.rect(screen, self.color, (self.x, self.y, self.width, self.height))
        pygame.draw.rect(screen, BLUE, (self.x + 5, self.y + 10, 30, 25))
        pygame.draw.rect(screen, BLUE, (self.x + 5, self.y + 45, 30, 25))
        pygame.draw.circle(screen, BLACK, (self.x + 5, self.y + 15), 8)
        pygame.draw.circle(screen, BLACK, (self.x + self.width - 5, self.y + 15), 8)
        pygame.draw.circle(screen, BLACK, (self.x + 5, self.y + self.height - 15), 8)
        pygame.draw.circle(screen, BLACK, (self.x + self.width - 5, self.y + self.height - 15), 8)
        
    def get_rect(self):
        return pygame.Rect(self.x, self.y, self.width, self.height)

class Obstacle:
    def __init__(self):
        self.width = 40
        self.height = 80
        self.x = random.randint(100, WIDTH - 140)
        self.y = -100
        self.speed = random.randint(3, 7)
        colors = [BLUE, GREEN, YELLOW, (255, 128, 0)]
        self.color = random.choice(colors)
        
    def update(self):
        self.y += self.speed
        
    def draw(self, screen):
        pygame.draw.rect(screen, self.color, (self.x, self.y, self.width, self.height))
        pygame.draw.rect(screen, BLACK, (self.x + 5, self.y + 10, 30, 25))
        pygame.draw.circle(screen, BLACK, (self.x + 5, self.y + 15), 6)
        pygame.draw.circle(screen, BLACK, (self.x + self.width - 5, self.y + 15), 6)
        
    def get_rect(self):
        return pygame.Rect(self.x, self.y, self.width, self.height)
        
    def is_off_screen(self):
        return self.y > HEIGHT

def read_potentiometer(ser):
    if ser is None:
        return 2048
    
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            match = re.search(r'ADC:\s*(\d+)', line)
            if match:
                return int(match.group(1))
    except:
        pass
    return None

def main():
    ser = connect_serial()
    
    car = Car()
    obstacles = []
    score = 0
    lives = 3
    game_over = False
    pot_value = 2048
    spawn_timer = 0
    speed_multiplier = 1.0
    
    road_lines = []
    for i in range(10):
        road_lines.append(i * 80)
    
    running = True
    while running:
        clock.tick(FPS)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r and game_over:
                    car = Car()
                    obstacles = []
                    score = 0
                    lives = 3
                    game_over = False
                    speed_multiplier = 1.0
        
        if not game_over:
            new_value = read_potentiometer(ser)
            if new_value is not None:
                pot_value = new_value
            
            car.update(pot_value)
            
            spawn_timer += 1
            if spawn_timer > max(30, 60 - int(score / 10)):
                obstacles.append(Obstacle())
                spawn_timer = 0
            
            for obs in obstacles[:]:
                obs.update()
                if obs.is_off_screen():
                    obstacles.remove(obs)
                    score += 1
                    if score % 10 == 0:
                        speed_multiplier += 0.1
                        
                if car.get_rect().colliderect(obs.get_rect()):
                    obstacles.remove(obs)
                    lives -= 1
                    if lives <= 0:
                        game_over = True
            
            for i in range(len(road_lines)):
                road_lines[i] += 5 * speed_multiplier
                if road_lines[i] > HEIGHT:
                    road_lines[i] = -80
        
        screen.fill(GREEN)

        pygame.draw.rect(screen, GRAY, (100, 0, WIDTH - 200, HEIGHT))

        for y in road_lines:
            pygame.draw.rect(screen, WHITE, (WIDTH // 2 - 5, y, 10, 60))

        pygame.draw.line(screen, WHITE, (100, 0), (100, HEIGHT), 5)
        pygame.draw.line(screen, WHITE, (WIDTH - 100, 0), (WIDTH - 100, HEIGHT), 5)

        for obs in obstacles:
            obs.draw(screen)

        car.draw(screen)

        score_text = font.render(f"Рахунок: {score}", True, BLACK)
        screen.blit(score_text, (10, 10))
        
        lives_text = font.render(f"Життя: {'❤️' * lives}", True, RED)
        screen.blit(lives_text, (10, 50))
        
        pot_text = small_font.render(f"ADC: {pot_value}", True, BLACK)
        screen.blit(pot_text, (WIDTH - 150, 10))
        
        speed_text = small_font.render(f"Швидкість: x{speed_multiplier:.1f}", True, BLACK)
        screen.blit(speed_text, (WIDTH - 200, 40))

        if game_over:
            overlay = pygame.Surface((WIDTH, HEIGHT))
            overlay.set_alpha(200)
            overlay.fill(BLACK)
            screen.blit(overlay, (0, 0))
            
            game_over_text = font.render("GAME OVER!", True, RED)
            final_score = font.render(f"Твій рахунок: {score}", True, WHITE)
            restart_text = small_font.render("Натисни R щоб перезапустити", True, WHITE)
            
            screen.blit(game_over_text, (WIDTH // 2 - 100, HEIGHT // 2 - 60))
            screen.blit(final_score, (WIDTH // 2 - 120, HEIGHT // 2))
            screen.blit(restart_text, (WIDTH // 2 - 150, HEIGHT // 2 + 60))
        
        pygame.display.flip()
    
    if ser:
        ser.close()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    print("=== STM32 Racing Game ===")
    print("Ухиляйся від інших машин!")
    print("\nПереконайся що STM32 підключена і відправляє дані...")
    input("Натисни Enter щоб почати...")
    main()
