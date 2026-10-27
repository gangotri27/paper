#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sqlite3
import csv
from datetime import datetime

class CSVExporter(Node):
    def __init__(self):
        super().__init__('csv_exporter')
        
        self.declare_parameter('db_path', '/tmp/omx_benchmark_results.db')
        self.declare_parameter('output_dir', '/tmp')
        
        self.db_path = self.get_parameter('db_path').value
        self.output_dir = self.get_parameter('output_dir').value
        
    def export_results(self):
        """Export benchmark results to CSV"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        conn = sqlite3.connect(self.db_path)
        
        # Export detailed results
        results_file = f'{self.output_dir}/benchmark_results_{timestamp}.csv'
        with open(results_file, 'w') as f:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM benchmark_results")
            
            # Get column names
            col_names = [description[0] for description in cursor.description]
            
            writer = csv.writer(f)
            writer.writerow(col_names)
            writer.writerows(cursor.fetchall())
        
        # Export summary
        summary_file = f'{self.output_dir}/benchmark_summary_{timestamp}.csv'
        with open(summary_file, 'w') as f:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM benchmark_summary")
            
            col_names = [description[0] for description in cursor.description]
            
            writer = csv.writer(f)
            writer.writerow(col_names)
            writer.writerows(cursor.fetchall())
        
        conn.close()
        
        self.get_logger().info(f'Results exported to: {results_file}')
        self.get_logger().info(f'Summary exported to: {summary_file}')
        
        return results_file, summary_file

def main(args=None):
    rclpy.init(args=args)
    
    exporter = CSVExporter()
    exporter.export_results()
    
    exporter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()